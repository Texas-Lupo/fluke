#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define LISTEN_PORT 9999
#define BUFFER_SIZE 2048
#define OVERHEAD_RATIO 0.25
#define SAFETY_MARGIN 0.5

// --- Telemetry Logging & OSD Defines ---
#define LOG_FILE_PATH "/etc/qlink.log"
#define OSD_FILE_PATH "/tmp/MSPOSD.msg"

#define OFFSET_EVM1  2
#define OFFSET_EVM2  2
#define OFFSET_RSSI1 5
#define OFFSET_SNR1  2

#define UPLINK_STABILITY_TICKS 80
#define CHANGE_COOLDOWN_TICKS  80  // 80 ticks * 25ms = 2 seconds of pure lockout

#define CMD_DELAY_US 50000 // 50ms pacing between commands

#define RAW_PWR_MCS0 2900
#define RAW_PWR_MCS1 2750
#define RAW_PWR_MCS2 2500
#define RAW_PWR_MCS3 2250
#define RAW_PWR_MCS4 1900
#define RAW_PWR_MCS5 1900
#define RAW_PWR_MCS6 1900
#define RAW_PWR_MCS7 1900

// --- 1. Structs and Enums ---

typedef struct {
    int transmitted_time;
    int evm1;
    int evm2;
    int recovered;
    int lost_packets;
    int rssi1;
    int snr1;
    int num_antennas;
    int noise_pnlty;
    int fec_change;
    char idr_code[16];
} GsTelemetry;

typedef enum {
    MSG_TYPE_INVALID = 0,
    MSG_TYPE_TELEMETRY,
    MSG_TYPE_SPECIAL
} GsMessageType;

typedef struct {
    int mcs;
    int bitrate;   
    int feck;
    int fecn;
    int bw;
    int roi;
    int gi;        
} downlink;

typedef struct {
    bool valid;
    int evm1;
    int evm2;
    int rssi1;
    int snr1;
} LinkThreshold;

typedef struct {
    GsTelemetry telemetry;
    downlink dl;
    bool has_new_data;
    int uplink_stable_count;   
    int cooldown_ticks;
    pthread_mutex_t lock;
} SharedData;

const int RAW_TX_POWER_TABLE[8] = {
    RAW_PWR_MCS0, RAW_PWR_MCS1, RAW_PWR_MCS2, RAW_PWR_MCS3,
    RAW_PWR_MCS4, RAW_PWR_MCS5, RAW_PWR_MCS6, RAW_PWR_MCS7
};

// --- 2. Bitrate Calculation Math ---

const float BASE_RATES[5][2][8] = {
    { {1.5,  3.0,  4.5,  6.0,  9.0, 12.0, 13.5, 15.0}, {1.7,  3.4,  5.1,  6.8, 10.2, 13.6, 15.0, 16.7} },
    { {3.0,  6.0,  9.0, 12.0, 18.0, 24.0, 27.0, 30.0}, {3.3,  6.8, 10.2, 13.6, 20.4, 27.2, 30.0, 33.4} },
    { {6.5, 13.0, 19.5, 26.0, 39.0, 52.0, 58.5, 65.0}, {7.2, 14.4, 21.7, 28.9, 43.3, 57.8, 65.0, 72.2} },
    { {13.5, 27.0, 40.5, 54.0, 81.0, 108.0, 121.5, 135.0}, {15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0} },
    { {27.0, 54.0, 81.0, 108.0, 162.0, 216.0, 243.0, 270.0}, {30.0, 60.0, 90.0, 120.0, 180.0, 240.0, 270.0, 300.0} }
};

void apply_link_settings(const downlink* d) {
    static downlink prev = { .mcs = -1, .bitrate = -1, .feck = -1, .fecn = -1, .bw = -1, .gi = -1 };
    char cmd[256];

    int mcs_safe = (d->mcs >= 0 && d->mcs <= 7) ? d->mcs : 0;
    bool is_uplink = (prev.mcs != -1) && (d->mcs > prev.mcs);

    if (is_uplink) {
        // --- UPLINK ORDER ---
        
        // 1. Power
        if (d->mcs != prev.mcs) {
            snprintf(cmd, sizeof(cmd), "iw dev wlan0 set txpower fixed %d", RAW_TX_POWER_TABLE[mcs_safe]);
            system(cmd);
        }
        // 2. Radio Pipe (Corrected flags: -B -G -S -L -M)
        if (d->mcs != prev.mcs || d->bw != prev.bw || d->gi != prev.gi) {
            const char* gi_str = (d->gi == 1) ? "short" : "long";
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_radio -B %d -G %s -S 1 -L 1 -M %d", d->bw, gi_str, d->mcs);
            system(cmd);
            usleep(CMD_DELAY_US);
        }
        // 3. FEC (Corrected flags: -k -n)
        if (d->feck != prev.feck || d->fecn != prev.fecn) {
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_fec -k %d -n %d", d->feck, d->fecn);
            system(cmd);
            usleep(10000);
        }
        // 4. Bitrate (Majestic REST API)
        if (d->bitrate != prev.bitrate) {
            snprintf(cmd, sizeof(cmd), "curl -s 'http://127.0.0.1/api/v1/set?video0.bitrate=%d'", d->bitrate);
            system(cmd);
        }
        
    } else {
        // --- DOWNLINK ORDER ---
        
        // 1. Bitrate (Majestic REST API)
        if (d->bitrate != prev.bitrate) {
            snprintf(cmd, sizeof(cmd), "curl -s 'http://127.0.0.1/api/v1/set?video0.bitrate=%d'", d->bitrate);
            system(cmd);
            usleep(CMD_DELAY_US);
        }
        // 2. FEC (Corrected flags: -k -n)
        if (d->feck != prev.feck || d->fecn != prev.fecn) {
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_fec -k %d -n %d", d->feck, d->fecn);
            system(cmd);
            usleep(CMD_DELAY_US);
        }
        // 3. Radio Pipe (Corrected flags: -B -G -S -L -M)
        if (d->mcs != prev.mcs || d->bw != prev.bw || d->gi != prev.gi) {
            const char* gi_str = (d->gi == 1) ? "short" : "long";
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_radio -B %d -G %s -S 1 -L 1 -M %d", d->bw, gi_str, d->mcs);
            system(cmd);
        }
        // 4. Power
        if (d->mcs != prev.mcs && prev.mcs != -1) {
            snprintf(cmd, sizeof(cmd), "iw dev wlan0 set txpower fixed %d", RAW_TX_POWER_TABLE[mcs_safe]);
            system(cmd);
        }
    }

    prev = *d;
    printf(">> [SYSTEM] Applied -> MCS:%d | BW:%d | GI:%s | FEC:%d/%d | Bitrate:%dkbps | Pwr:%d\n",
           d->mcs, d->bw, (d->gi == 1) ? "short" : "long", d->feck, d->fecn, d->bitrate, RAW_TX_POWER_TABLE[mcs_safe]);
}

int calculate_safe_bitrate(int mcs, int bw, int fec_k, int fec_n, int gi, float overhead_ratio, float safety_margin) {
    if (mcs < 0 || mcs > 7) return 0;
    if (gi < 0 || gi > 1) return 0;

    int bw_idx = -1;
    switch(bw) {
        case 5:  bw_idx = 0; break;
        case 10: bw_idx = 1; break;
        case 20: bw_idx = 2; break;
        case 40: bw_idx = 3; break;
        case 80: bw_idx = 4; break;
        default: return 0;
    }

    float base_rate = BASE_RATES[bw_idx][gi][mcs];
    float unusable = base_rate * overhead_ratio;
    float usable = base_rate - unusable;
    float fec_overhead_ratio = 0.0f;
    
    if (fec_n > 0 && fec_k <= fec_n) {
        fec_overhead_ratio = (float)(fec_n - fec_k) / (float)fec_n;
    }

    float fec_overhead = usable * fec_overhead_ratio;
    float safe_video_mbps = (usable - fec_overhead) * safety_margin;

    return (int)(safe_video_mbps * 1000.0f);
}

void update_downlink_bitrate(downlink* d) {
    d->bitrate = calculate_safe_bitrate(d->mcs, d->bw, d->feck, d->fecn, d->gi, OVERHEAD_RATIO, SAFETY_MARGIN);
}

void downlink_init(downlink* d) {
    d->mcs = 5;
    d->feck = 9;
    d->fecn = 12;
    d->bw = 20;
    d->roi = 0;
    d->gi = 0;
    update_downlink_bitrate(d);
    apply_link_settings(d);
}


// --- 3. Parser Functions ---

bool parse_telemetry_string(const char *msg, GsTelemetry *telemetry) {
    if (!msg || !telemetry) return false;
    memset(telemetry, 0, sizeof(GsTelemetry));
    telemetry->evm1 = 0;
    telemetry->evm2 = 0;
    telemetry->rssi1 = -105;

    char *msg_copy = strdup(msg);
    if (!msg_copy) return false;

    char *token = strtok(msg_copy, ":");
    int index = 0;
    while (token != NULL) {
        switch (index) {
            case 0: telemetry->transmitted_time = atoi(token); break;
            case 1: telemetry->evm1 = atoi(token); break;
            case 2: telemetry->evm2 = atoi(token); break;
            case 3: telemetry->recovered = atoi(token); break;
            case 4: telemetry->lost_packets = atoi(token); break;
            case 5: telemetry->rssi1 = atoi(token); break;
            case 6: telemetry->snr1 = atoi(token); break;
            case 7: telemetry->num_antennas = atoi(token); break;
            case 8: telemetry->noise_pnlty = atoi(token); break;
            case 9: telemetry->fec_change = atoi(token); break;
            case 10:
                strncpy(telemetry->idr_code, token, sizeof(telemetry->idr_code) - 1);
                telemetry->idr_code[sizeof(telemetry->idr_code) - 1] = '\0';
                break;
            default: break;
        }
        token = strtok(NULL, ":");
        index++;
    }
    free(msg_copy);
    return (index >= 10);
}

GsMessageType parse_gs_packet(const uint8_t *buffer, size_t buffer_len, GsTelemetry *telemetry_out, char *special_cmd_out, size_t special_cmd_max) {
    if (buffer_len < sizeof(uint32_t)) return MSG_TYPE_INVALID;
    uint32_t msg_length;
    memcpy(&msg_length, buffer, sizeof(msg_length));
    msg_length = ntohl(msg_length);
    if (msg_length > (buffer_len - sizeof(uint32_t))) return MSG_TYPE_INVALID;

    const char *payload = (const char *)(buffer + sizeof(uint32_t));
    if (strncmp(payload, "special:", 8) == 0) {
        if (special_cmd_out) {
            strncpy(special_cmd_out, payload, special_cmd_max - 1);
            special_cmd_out[special_cmd_max - 1] = '\0';
        }
        return MSG_TYPE_SPECIAL;
    }
    if (parse_telemetry_string(payload, telemetry_out)) return MSG_TYPE_TELEMETRY;
    return MSG_TYPE_INVALID;
}


// --- 4. Threads ---

void* udp_listener_thread(void* arg) {
    SharedData *shared = (SharedData *)arg;

    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    uint8_t buffer[BUFFER_SIZE];

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        pthread_exit(NULL);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(LISTEN_PORT);

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        pthread_exit(NULL);
    }

    printf("Listening for GS telemetry on UDP port %d...\n", LISTEN_PORT);

    char special_cmd[256];
    GsTelemetry temp_telemetry;

    while (1) {
        int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                         (struct sockaddr *)&client_addr, &client_addr_len);
        if (n < 0) break;

        buffer[n] = '\0';
        GsMessageType msg_type = parse_gs_packet(buffer, n, &temp_telemetry, special_cmd, sizeof(special_cmd));

        if (msg_type == MSG_TYPE_TELEMETRY) {
            pthread_mutex_lock(&shared->lock);
            shared->telemetry = temp_telemetry;
            shared->has_new_data = true;
            pthread_mutex_unlock(&shared->lock);
        }
    }

    close(sockfd);
    return NULL;
}

// OSD thread writes formatting to MSPOSD file exactly like original alink
void* osd_writer_thread(void* arg) {
    SharedData *shared = (SharedData *)arg;
    char osd_msg[256];

    while (1) {
        pthread_mutex_lock(&shared->lock);
        int mcs = shared->dl.mcs;
        int bitrate = shared->dl.bitrate;
        int feck = shared->dl.feck;
        int fecn = shared->dl.fecn;
        int lost = shared->telemetry.lost_packets;
        int snr = shared->telemetry.snr1;
        int cooldown = shared->cooldown_ticks;
        pthread_mutex_unlock(&shared->lock);

        // Uses MSPOSD tags: &L40 (Line) &F25 (Font Size)
        snprintf(osd_msg, sizeof(osd_msg), "&L40&F25 QL: MCS%d %dkbps FEC%d/%d SNR%d L:%d %s",
                 mcs, bitrate, feck, fecn, snr, lost, cooldown > 0 ? "[WAIT]" : "[RDY]");

        FILE *file = fopen(OSD_FILE_PATH, "w");
        if (file) {
            fwrite(osd_msg, sizeof(char), strlen(osd_msg), file);
            fclose(file);
        }
        
        sleep(1); // Update OSD every second
    }
    return NULL;
}

// --- 5. Log File Management ---

void load_logs(LinkThreshold *th) {
    for (int i = 0; i <= 7; i++) {
        th[i].valid = false;
    }

    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (!f) return; 

    int mcs, valid, evm1, evm2, rssi1, snr1;
    while (fscanf(f, "%d %d %d %d %d %d", &mcs, &valid, &evm1, &evm2, &rssi1, &snr1) == 6) {
        if (mcs >= 0 && mcs <= 7) {
            th[mcs].valid = valid;
            th[mcs].evm1 = evm1;
            th[mcs].evm2 = evm2;
            th[mcs].rssi1 = rssi1;
            th[mcs].snr1 = snr1;
        }
    }
    fclose(f);
}

void save_logs(LinkThreshold *th) {
    FILE *f = fopen(LOG_FILE_PATH, "w");
    if (!f) return;
    for (int i = 0; i <= 7; i++) {
        fprintf(f, "%d %d %d %d %d %d\n", i, th[i].valid, th[i].evm1, th[i].evm2, th[i].rssi1, th[i].snr1);
    }
    fclose(f);
}

// --- 6. Main Link Logic ---

bool g0ylink(const GsTelemetry* t, downlink* d, LinkThreshold* th, int* stable_count, int* cooldown_ticks) {
    int old_mcs = d->mcs;
    bool trigger_downlink = false;
    bool can_uplink_now = false;

    // --- 1. DOWNLINK CHECK ---
    if (th[old_mcs].valid) {
        int bad_evm1 = th[old_mcs].evm1 - OFFSET_EVM1;
        int bad_evm2 = th[old_mcs].evm2 - OFFSET_EVM2;
        int bad_rssi1 = th[old_mcs].rssi1 - OFFSET_RSSI1;
        int bad_snr1 = th[old_mcs].snr1 - OFFSET_SNR1;

        if (t->evm1 >= bad_evm1 || t->evm2 >= bad_evm2 ||
            t->rssi1 <= bad_rssi1 || t->snr1 <= bad_snr1 ||
            t->lost_packets > 0) {
            trigger_downlink = true;
        }
    } else {
        if (t->lost_packets > 0) trigger_downlink = true;
    }

    if (trigger_downlink && d->mcs > 0) {
        th[old_mcs].valid = true;
        th[old_mcs].evm1 = t->evm1 + OFFSET_EVM1;
        th[old_mcs].evm2 = t->evm2 + OFFSET_EVM2;
        th[old_mcs].rssi1 = t->rssi1 + OFFSET_RSSI1;
        th[old_mcs].snr1 = t->snr1 + OFFSET_SNR1;

        d->mcs--;
        *stable_count = 0; 
        *cooldown_ticks = CHANGE_COOLDOWN_TICKS; 
        
        update_downlink_bitrate(d);
        apply_link_settings(d);
        return true;
    }

    // --- 2. UPLINK CHECK ---
    if (!trigger_downlink && d->mcs < 7) {
        int target_mcs = d->mcs + 1;
        bool conditions_met = false;

        if (th[target_mcs].valid) {
            if (t->evm1 < th[target_mcs].evm1 && t->evm2 < th[target_mcs].evm2 &&
                t->rssi1 > th[target_mcs].rssi1 && t->snr1 > th[target_mcs].snr1 &&
                t->lost_packets == 0) {
                conditions_met = true;
            }
        } else {
            if (t->snr1 > 25 && t->lost_packets == 0) {
                conditions_met = true;
            }
        }

        if (conditions_met) {
            (*stable_count)++;
            if (*stable_count >= UPLINK_STABILITY_TICKS) {
                can_uplink_now = true;
                *stable_count = 0; 
            }
        } else {
            *stable_count = 0; 
        }
    } else {
        *stable_count = 0; 
    }

    if (can_uplink_now) {
        d->mcs++;
        *cooldown_ticks = CHANGE_COOLDOWN_TICKS; 

        update_downlink_bitrate(d);
        apply_link_settings(d);
    }

    return false;
}

// --- 7. Main ---

int main(void) {
    SharedData shared;
    memset(&shared, 0, sizeof(shared));

    shared.has_new_data = false;
    shared.cooldown_ticks = 0;
    downlink_init(&shared.dl);

    LinkThreshold thresholds[8];
    load_logs(thresholds);

    if (pthread_mutex_init(&shared.lock, NULL) != 0) {
        perror("Mutex init failed");
        return EXIT_FAILURE;
    }

    pthread_t listener_tid, osd_tid;
    pthread_create(&listener_tid, NULL, udp_listener_thread, &shared);
    pthread_create(&osd_tid, NULL, osd_writer_thread, &shared);

    // Main Control Loop running every 25ms (40Hz)
    while (1) {
        bool needs_save = false;

        pthread_mutex_lock(&shared.lock);
        
        // 1. ALWAYS decrement cooldown timer based on real time, regardless of new data
        if (shared.cooldown_ticks > 0) {
            shared.cooldown_ticks--;
        }

        // 2. Only process new telemetry if the hardware has finished its cooldown
        if (shared.has_new_data) {
            if (shared.cooldown_ticks == 0) {
                needs_save = g0ylink(&shared.telemetry, &shared.dl, thresholds, &shared.uplink_stable_count, &shared.cooldown_ticks);
            }
            shared.has_new_data = false;
        }
        
        pthread_mutex_unlock(&shared.lock);

        if (needs_save) save_logs(thresholds);

        usleep(25000); // 25ms
    }

    pthread_join(listener_tid, NULL);
    pthread_join(osd_tid, NULL);
    pthread_mutex_destroy(&shared.lock);
    return EXIT_SUCCESS;
}
