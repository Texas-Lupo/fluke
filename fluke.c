#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <math.h>

#define LISTEN_PORT 9999
#define BUFFER_SIZE 2048
#define OVERHEAD_RATIO 0.25
#define SAFETY_MARGIN 0.65

// --- Telemetry Logging & Hysteresis ---
#define LOG_FILE_PATH "/etc/fluke.log"

#define OFFSET_EVM1  2
#define OFFSET_EVM2  2
#define OFFSET_RSSI1 1
#define OFFSET_SNR1  1

#define UPLINK_STABILITY_TICKS 60
#define CHANGE_COOLDOWN_TICKS  60 
#define CMD_DELAY_US 9000
#define NO_TELEMETRY_TICKS_THRESH 80  // 2 seconds without data triggers failsafe

// --- Threshold Defines ---
#define DOWNLINK_LOST_PKTS_THRESH 2   
#define UPLINK_LOST_PKTS_THRESH   0   

#define FEC_N_CONSTANT            12
#define FEC_K_HIGH_PROTECTION     8   // 8/12
#define FEC_K_MED_PROTECTION      9   // 9/12
#define FEC_K_LOW_PROTECTION      10  // 10/12
#define FEC_K_FAILSAFE            2   // 2/12

#define FEC_RECOVERED_THRESH_HIGH 4   
#define FEC_RECOVERED_THRESH_LOW  1   

// --- Raw TX Power ---
#define RAW_PWR_MCS0 2750
#define RAW_PWR_MCS1 2500
#define RAW_PWR_MCS2 2250
#define RAW_PWR_MCS3 2000
#define RAW_PWR_MCS4 1400
#define RAW_PWR_MCS5 450
#define RAW_PWR_MCS6 100
#define RAW_PWR_MCS7 100

// --- OSD Globals ---
int osd_level = 4;
int set_osd_font_size = 20;
int set_osd_colour = 7;
typedef struct {
    int udp_out_sock;
    char udp_out_ip[INET_ADDRSTRLEN];
    int udp_out_port;
} osd_udp_config_t;

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

// Shared State struct for safe multithreading
typedef struct {
    GsTelemetry telemetry;
    downlink dl;
    bool has_new_data;
    int uplink_stable_count;   
    int cooldown_ticks;
    int no_telemetry_ticks;
    
    // Operating Modes
    bool legacy_mode;
    bool failsafe_mode;
    bool log_corrupted;
    
    osd_udp_config_t osd_config;
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
        if (d->mcs != prev.mcs) {
            snprintf(cmd, sizeof(cmd), "iw dev wlan0 set txpower fixed %d", RAW_TX_POWER_TABLE[mcs_safe]);
            system(cmd); usleep(CMD_DELAY_US);
        }
        if (d->mcs != prev.mcs || d->bw != prev.bw || d->gi != prev.gi) {
            const char* gi_str = (d->gi == 1) ? "short" : "long";
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_radio -B %d -G %s -S 1 -L 1 -M %d", d->bw, gi_str, d->mcs);
            system(cmd); usleep(CMD_DELAY_US);
        }
        if (d->feck != prev.feck || d->fecn != prev.fecn) {
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_fec -k %d -n %d", d->feck, d->fecn);
            system(cmd); usleep(CMD_DELAY_US);
        }
        if (d->bitrate != prev.bitrate) {
            snprintf(cmd, sizeof(cmd), "curl -s 'http://127.0.0.1/api/v1/set?video0.bitrate=%d'", d->bitrate);
            system(cmd); usleep(CMD_DELAY_US);
        }
    } else {
        if (d->bitrate != prev.bitrate) {
            snprintf(cmd, sizeof(cmd), "curl -s 'http://127.0.0.1/api/v1/set?video0.bitrate=%d'", d->bitrate);
            system(cmd); usleep(CMD_DELAY_US);
        }
        if (d->feck != prev.feck || d->fecn != prev.fecn) {
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_fec -k %d -n %d", d->feck, d->fecn);
            system(cmd); usleep(CMD_DELAY_US);
        }
        if (d->mcs != prev.mcs || d->bw != prev.bw || d->gi != prev.gi) {
            const char* gi_str = (d->gi == 1) ? "short" : "long";
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_radio -B %d -G %s -S 1 -L 1 -M %d", d->bw, gi_str, d->mcs);
            system(cmd); usleep(CMD_DELAY_US);
        }
        if (d->mcs != prev.mcs && prev.mcs != -1) {
            snprintf(cmd, sizeof(cmd), "iw dev wlan0 set txpower fixed %d", RAW_TX_POWER_TABLE[mcs_safe]);
            system(cmd); usleep(CMD_DELAY_US);
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
    float max_app_mbps = usable - fec_overhead;
    return (int)((max_app_mbps * safety_margin) * 1000.0f);
}

void update_downlink_bitrate(downlink* d) {
    d->bitrate = calculate_safe_bitrate(d->mcs, d->bw, d->feck, d->fecn, d->gi, OVERHEAD_RATIO, SAFETY_MARGIN);
}

void downlink_init(downlink* d) {
    d->mcs = 5;
    d->feck = FEC_K_MED_PROTECTION;
    d->fecn = FEC_N_CONSTANT;
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

// --- 4. Log Validation & Management ---

// Ensures that higher MCS index requires better signal quality than a lower MCS index
bool validate_logs(LinkThreshold *th, bool legacy_mode) {
    for (int i = 0; i < 7; i++) {
        if (!th[i].valid) continue;
        for (int j = i + 1; j <= 7; j++) {
            if (!th[j].valid) continue;

            // Corruption: Identical thresholds for different MCS
            if (th[i].rssi1 == th[j].rssi1 && th[i].snr1 == th[j].snr1 &&
                (legacy_mode || (th[i].evm1 == th[j].evm1 && th[i].evm2 == th[j].evm2))) {
                return false; 
            }

            // Corruption: Higher MCS requires worse/equal signal than lower MCS
            if (th[j].rssi1 < th[i].rssi1) return false;
            if (th[j].snr1 < th[i].snr1) return false;
            
            if (!legacy_mode) {
                if (th[j].evm1 > th[i].evm1) return false;
                if (th[j].evm2 > th[i].evm2) return false;
            }
        }
    }
    return true;
}

void load_logs(LinkThreshold *th, bool *legacy_mode) {
    for (int i = 0; i <= 7; i++) th[i].valid = false;
    
    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (!f) return;

    char header[32];
    int legacy_val = 0;
    
    // Check for new header format
    if (fscanf(f, "%s %d", header, &legacy_val) == 2 && strcmp(header, "LEGACY") == 0) {
        *legacy_mode = (legacy_val == 1);
    } else {
        rewind(f); // Fallback for old log files without header
    }

    int mcs, valid, evm1, evm2, rssi1, snr1;
    while (fscanf(f, "%d %d %d %d %d %d", &mcs, &valid, &evm1, &evm2, &rssi1, &snr1) == 6) {
        if (mcs >= 0 && mcs <= 7) {
            th[mcs].valid = valid;
            th[mcs].evm1 = evm1; th[mcs].evm2 = evm2;
            th[mcs].rssi1 = rssi1; th[mcs].snr1 = snr1;
        }
    }
    fclose(f);
    printf(">> Loaded historical MCS thresholds from %s (Legacy: %d)\n", LOG_FILE_PATH, *legacy_mode);
}

void save_logs(LinkThreshold *th, bool legacy_mode) {
    FILE *f = fopen(LOG_FILE_PATH, "w");
    if (!f) {
        perror("Failed to save history log");
        return;
    }
    fprintf(f, "LEGACY %d\n", legacy_mode ? 1 : 0);
    for (int i = 0; i <= 7; i++) {
        fprintf(f, "%d %d %d %d %d %d\n", i, th[i].valid, th[i].evm1, th[i].evm2, th[i].rssi1, th[i].snr1);
    }
    fclose(f);
}


// --- 5. Thread Workers ---

void* periodic_update_osd(void* arg) {
    SharedData *shared = (SharedData *)arg;
    struct sockaddr_in udp_out_addr;
    bool udp_enabled = (shared->osd_config.udp_out_sock != -1);

    if (udp_enabled) {
        memset(&udp_out_addr, 0, sizeof(udp_out_addr));
        udp_out_addr.sin_family = AF_INET;
        udp_out_addr.sin_port = htons(shared->osd_config.udp_out_port);
        inet_pton(AF_INET, shared->osd_config.udp_out_ip, &udp_out_addr.sin_addr);
    }

    while (true) {
        sleep(1);
        if (osd_level == 0) continue;

        // Safely extract variables for OSD
        pthread_mutex_lock(&shared->lock);
        int mcs = shared->dl.mcs;
        int feck = shared->dl.feck;
        int fecn = shared->dl.fecn;
        int evm = shared->telemetry.evm1;
        int rssi = shared->telemetry.rssi1;
        int snr = shared->telemetry.snr1;
        int lost = shared->telemetry.lost_packets;
        int rec = shared->telemetry.recovered;
        int cooldown = shared->cooldown_ticks;
        
        bool legacy = shared->legacy_mode;
        bool failsafe = shared->failsafe_mode;
        bool corrupted = shared->log_corrupted;
        pthread_mutex_unlock(&shared->lock);

        char rdy_str[16];
        if (cooldown > 0) strcpy(rdy_str, "WAIT");
        else strcpy(rdy_str, "RDY");

        char mode_str[128];
        snprintf(mode_str, sizeof(mode_str), "Modes: %s / %s %s", 
            legacy ? "LEGACY" : "STANDARD",
            failsafe ? "FAILSAFE" : "NORMAL",
            corrupted ? "[CORRUPTED LOG]" : "");

        char full_osd_string[512];
        snprintf(full_osd_string, sizeof(full_osd_string), 
            "&L%d0&F%d MCS:%d FEC:%d/%d EVM:%d RSSI:%d SNR:%d LST:%d REC:%d [%s]\n%s",
            set_osd_colour, set_osd_font_size, mcs, feck, fecn, evm, rssi, snr, lost, rec, rdy_str, mode_str);

        if (udp_enabled) {
            sendto(shared->osd_config.udp_out_sock, full_osd_string, strlen(full_osd_string), 0,
                   (struct sockaddr *)&udp_out_addr, sizeof(udp_out_addr));
        } else {
            FILE *file = fopen("/tmp/MSPOSD.msg", "w");
            if (file) {
                fwrite(full_osd_string, sizeof(char), strlen(full_osd_string), file);
                fclose(file);
            }
        }
    }
    return NULL;
}

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

    printf("Listening for GS telemetry on UDP port %d...\n\n", LISTEN_PORT);

    char special_cmd[256];
    GsTelemetry temp_telemetry;

    while (1) {
        int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                         (struct sockaddr *)&client_addr, &client_addr_len);
        if (n < 0) break;

        buffer[n] = '\0';
        GsMessageType msg_type = parse_gs_packet(buffer, n, &temp_telemetry, special_cmd, sizeof(special_cmd));

        switch (msg_type) {
            case MSG_TYPE_TELEMETRY:
                pthread_mutex_lock(&shared->lock);
                shared->telemetry = temp_telemetry;
                shared->has_new_data = true;
                
                // Legacy latch check
                if (temp_telemetry.evm1 > 45 || temp_telemetry.evm2 > 45) {
                    shared->legacy_mode = true;
                }
                pthread_mutex_unlock(&shared->lock);
                break;

            case MSG_TYPE_SPECIAL:
                printf(">> SPECIAL COMMAND RECEIVED: %s\n\n", special_cmd);
                if (strncmp(special_cmd, "special:request_keyframe", 24) == 0) {
                    system("curl -s 'http://127.0.0.1/request/idr'");
                    printf(">> [SYSTEM] Keyframe requested via Majestic API\n");
                }
                break;

            case MSG_TYPE_INVALID:
                break;
        }
    }
    close(sockfd);
    return NULL;
}


// --- 7. Main Link Logic ---

bool g0ylink(const GsTelemetry* t, downlink* d, LinkThreshold* th, SharedData* state) {
    bool log_saved = false;
    bool settings_changed = false;

    // --- FAILSAFE LOGIC ---
    if (state->failsafe_mode) {
        // Force settings
        if (d->mcs != 0 || d->feck != FEC_K_FAILSAFE) {
            d->mcs = 0;
            d->feck = FEC_K_FAILSAFE;
            update_downlink_bitrate(d);
            apply_link_settings(d);
        }

        // Check if conditions are good enough to escape to MCS 1
        bool can_escape = false;
        if (th[1].valid) {
            bool evm1_ok  = state->legacy_mode || t->evm1 == 0 || (t->evm1 < th[1].evm1);
            bool evm2_ok  = state->legacy_mode || t->evm2 == 0 || (t->evm2 < th[1].evm2);
            bool rssi1_ok = (t->rssi1 == 0) || (t->rssi1 > th[1].rssi1);
            bool snr1_ok  = (t->snr1 == 0)  || (t->snr1 > th[1].snr1);
            bool lost_ok  = (t->lost_packets <= UPLINK_LOST_PKTS_THRESH);

            if (evm1_ok && evm2_ok && rssi1_ok && snr1_ok && lost_ok) can_escape = true;
        } else {
            if ((t->snr1 == 0 || t->snr1 > 25) && t->lost_packets <= UPLINK_LOST_PKTS_THRESH) can_escape = true;
        }

        if (can_escape) {
            state->uplink_stable_count++;
            if (state->uplink_stable_count >= UPLINK_STABILITY_TICKS) {
                state->failsafe_mode = false;
                state->log_corrupted = false; // Resolved
                d->mcs = 1;
                state->cooldown_ticks = CHANGE_COOLDOWN_TICKS;
                state->uplink_stable_count = 0;
                settings_changed = true;
                printf("\n>> [g0ylink] EXIT FAILSAFE -> Normal Operation (MCS 1)\n");
            }
        } else {
            state->uplink_stable_count = 0;
        }
        
        if (settings_changed) {
            update_downlink_bitrate(d);
            apply_link_settings(d);
        }
        return false; // Do not manipulate logs while in failsafe
    }

    // --- NORMAL OPERATION ---

    // 0. VARIABLE FEC ADJUSTMENT
    int target_feck = d->feck;
    if (t->recovered >= FEC_RECOVERED_THRESH_HIGH) {
        target_feck = FEC_K_HIGH_PROTECTION; 
    } else if (t->recovered <= FEC_RECOVERED_THRESH_LOW) {
        target_feck = FEC_K_LOW_PROTECTION;  
    } else {
        target_feck = FEC_K_MED_PROTECTION;  
    }

    if (target_feck != d->feck) {
        printf("\n>> [g0ylink] FEC ADAPT: Recovered=%d -> Target FEC %d/%d\n", t->recovered, target_feck, d->fecn);
        d->feck = target_feck;
        settings_changed = true;
    }

    // COOLDOWN HARD-LOCK FOR MCS
    if (state->cooldown_ticks > 0) {
        state->cooldown_ticks--;
        if (settings_changed) {
            update_downlink_bitrate(d);
            apply_link_settings(d);
        }
        return false; 
    }

    int old_mcs = d->mcs;
    bool trigger_downlink = false;
    bool can_uplink_now = false;

    // 1. DOWNLINK CHECK
    if (th[old_mcs].valid) {
        int bad_evm1 = th[old_mcs].evm1 - OFFSET_EVM1;
        int bad_evm2 = th[old_mcs].evm2 - OFFSET_EVM2;
        int bad_rssi1 = th[old_mcs].rssi1 - OFFSET_RSSI1;
        int bad_snr1 = th[old_mcs].snr1 - OFFSET_SNR1;

        bool evm1_bad  = !state->legacy_mode && (t->evm1 != 0)  && (t->evm1 >= bad_evm1);
        bool evm2_bad  = !state->legacy_mode && (t->evm2 != 0)  && (t->evm2 >= bad_evm2);
        bool rssi1_bad = (t->rssi1 != 0) && (t->rssi1 <= bad_rssi1);
        bool snr1_bad  = (t->snr1 != 0)  && (t->snr1 <= bad_snr1);
        bool lost_bad  = (t->lost_packets >= DOWNLINK_LOST_PKTS_THRESH);

        if (evm1_bad || evm2_bad || rssi1_bad || snr1_bad || lost_bad) {
            trigger_downlink = true;
        }
    } else {
        if (t->lost_packets >= DOWNLINK_LOST_PKTS_THRESH) trigger_downlink = true;
    }

    // Secondary Failsafe Trigger
    if (t->lost_packets > 0 && d->mcs == 0 && d->feck <= FEC_K_HIGH_PROTECTION) {
        state->failsafe_mode = true;
        printf("\n>> [g0ylink] CRITICAL: Packet loss on MCS0/High FEC. Entering FAILSAFE.\n");
        return false;
    }

    if (trigger_downlink && d->mcs > 0) {
        LinkThreshold temp_th;
        temp_th.valid = true;
        temp_th.evm1 = t->evm1 + OFFSET_EVM1;
        temp_th.evm2 = t->evm2 + OFFSET_EVM2;
        temp_th.rssi1 = t->rssi1 + OFFSET_RSSI1;
        temp_th.snr1 = t->snr1 + OFFSET_SNR1;

        // Corruption Check on Save: Lower MCS cannot require better signal than higher MCS
        if (old_mcs < 7 && th[old_mcs + 1].valid) {
            bool is_corrupt = false;
            if (temp_th.rssi1 > th[old_mcs + 1].rssi1) is_corrupt = true;
            if (temp_th.snr1 > th[old_mcs + 1].snr1) is_corrupt = true;
            if (!state->legacy_mode) {
                if (temp_th.evm1 < th[old_mcs + 1].evm1) is_corrupt = true;
                if (temp_th.evm2 < th[old_mcs + 1].evm2) is_corrupt = true;
            }

            if (is_corrupt) {
                printf("\n>> [g0ylink] LOG CORRUPTION DETECTED ON SAVE. Deleting logs and entering FAILSAFE.\n");
                unlink(LOG_FILE_PATH);
                for(int i=0; i<=7; i++) th[i].valid = false;
                state->failsafe_mode = true;
                state->log_corrupted = true;
                return false;
            }
        }

        th[old_mcs] = temp_th;
        log_saved = true;

        d->mcs--;
        state->uplink_stable_count = 0; 
        state->cooldown_ticks = CHANGE_COOLDOWN_TICKS; 
        settings_changed = true;
    } 
    else if (!trigger_downlink && d->mcs < 7) {
        // 2. UPLINK CHECK
        int target_mcs = d->mcs + 1;
        bool conditions_met = false;

        if (th[target_mcs].valid) {
            bool evm1_ok  = state->legacy_mode || (t->evm1 == 0)  || (t->evm1 < th[target_mcs].evm1);
            bool evm2_ok  = state->legacy_mode || (t->evm2 == 0)  || (t->evm2 < th[target_mcs].evm2);
            bool rssi1_ok = (t->rssi1 == 0) || (t->rssi1 > th[target_mcs].rssi1);
            bool snr1_ok  = (t->snr1 == 0)  || (t->snr1 > th[target_mcs].snr1);
            bool lost_ok  = (t->lost_packets <= UPLINK_LOST_PKTS_THRESH);

            if (evm1_ok && evm2_ok && rssi1_ok && snr1_ok && lost_ok) conditions_met = true;
        } else {
            bool snr1_ok = (t->snr1 == 0) || (t->snr1 > 25);
            bool lost_ok = (t->lost_packets <= UPLINK_LOST_PKTS_THRESH);

            if (snr1_ok && lost_ok) conditions_met = true;
        }

        if (conditions_met) {
            state->uplink_stable_count++;
            if (state->uplink_stable_count >= UPLINK_STABILITY_TICKS) {
                can_uplink_now = true;
                state->uplink_stable_count = 0; 
            }
        } else {
            state->uplink_stable_count = 0; 
        }
    } else {
        state->uplink_stable_count = 0; 
    }

    if (can_uplink_now) {
        d->mcs++;
        state->cooldown_ticks = CHANGE_COOLDOWN_TICKS;
        settings_changed = true;
    }

    // --- APPLY CHANGES ONCE ---
    if (settings_changed) {
        update_downlink_bitrate(d);
        apply_link_settings(d);
        
        if (d->mcs < old_mcs) {
            printf("\n>> [g0ylink] DOWNLINK: %d -> %d | New Bitrate: %d Kbps\n", old_mcs, d->mcs, d->bitrate);
        } else if (d->mcs > old_mcs) {
            printf("\n>> [g0ylink] UPLINK: %d -> %d (Stable for 2s) | New Bitrate: %d Kbps\n", old_mcs, d->mcs, d->bitrate);
        }
    }

    return log_saved;
}


// --- 8. Main ---

int main(void) {
    SharedData shared;
    memset(&shared, 0, sizeof(shared));

    shared.has_new_data = false;
    shared.cooldown_ticks = 0;
    shared.no_telemetry_ticks = 0;
    shared.legacy_mode = false;
    shared.failsafe_mode = false;
    shared.log_corrupted = false;
    shared.osd_config.udp_out_sock = -1; 
    
    downlink_init(&shared.dl);

    LinkThreshold thresholds[8];
    load_logs(thresholds, &shared.legacy_mode);

    if (!validate_logs(thresholds, shared.legacy_mode)) {
        printf("\n>> [SYSTEM] LOG CORRUPTION DETECTED AT BOOT. Deleting logs and entering FAILSAFE.\n");
        unlink(LOG_FILE_PATH);
        for(int i=0; i<=7; i++) thresholds[i].valid = false;
        shared.failsafe_mode = true;
        shared.log_corrupted = true;
    }

    if (pthread_mutex_init(&shared.lock, NULL) != 0) {
        perror("Mutex init failed");
        return EXIT_FAILURE;
    }

    pthread_t listener_tid, osd_tid;
    
    if (pthread_create(&listener_tid, NULL, udp_listener_thread, &shared) != 0) {
        perror("Failed to create listener thread");
        return EXIT_FAILURE;
    }
    
    if (pthread_create(&osd_tid, NULL, periodic_update_osd, &shared) != 0) {
        perror("Failed to create OSD thread");
        return EXIT_FAILURE;
    }

    // Main Control Loop running every 25ms (40Hz)
    while (1) {
        bool needs_save = false;

        pthread_mutex_lock(&shared.lock);
        
        // Handle Telemetry Timeouts
        if (!shared.has_new_data) {
            shared.no_telemetry_ticks++;
            if (shared.no_telemetry_ticks >= NO_TELEMETRY_TICKS_THRESH && !shared.failsafe_mode) {
                shared.failsafe_mode = true;
                printf("\n>> [SYSTEM] NO TELEMETRY TIMEOUT. Entering FAILSAFE.\n");
            }
        } else {
            shared.no_telemetry_ticks = 0;
        }

        // Always run g0ylink (handles failsafe and FEC even without new data)
        needs_save = g0ylink(&shared.telemetry, &shared.dl, thresholds, &shared);
        shared.has_new_data = false;
        
        bool current_legacy_mode = shared.legacy_mode;
        pthread_mutex_unlock(&shared.lock);

        if (needs_save) {
            save_logs(thresholds, current_legacy_mode);
        }

        usleep(25000); // 25ms
    }

    pthread_join(listener_tid, NULL);
    pthread_join(osd_tid, NULL);
    pthread_mutex_destroy(&shared.lock);
    return EXIT_SUCCESS;
}
