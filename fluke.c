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
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

#define LISTEN_PORT 9999
#define BUFFER_SIZE 2048
#define FIFO_PATH "/tmp/qlink_cmd"
#define CONFIG_FILE_PATH "/etc/qlink.conf"
#define LOG_FILE_PATH "/etc/fluke.log"

// --- OSD Globals ---
typedef struct {
    int udp_out_sock;
    char udp_out_ip[INET_ADDRSTRLEN];
    int udp_out_port;
} osd_udp_config_t;

// --- CONFIGURATION STRUCT ---
typedef struct {
    float overhead_ratio;
    float safety_margin;
    int offset_evm1;
    int offset_evm2;
    int offset_rssi1;
    int offset_snr1;
    int uplink_stability_ticks;
    int change_cooldown_ticks;
    int cmd_delay_us;
    int no_telemetry_ticks_thresh;
    int downlink_lost_pkts_thresh;
    int downlink_lost_pkts67_thresh;
    int downlink_lost_pkts67_c_thresh;
    int uplink_lost_pkts_thresh;
    int fec_n_constant;
    int fec_k_probing_start;
    int fec_k_high_protection;
    int fec_k_med_protection;
    int fec_k_low_protection;
    int fec_k_failsafe;
    int fec_recovered_thresh_high;
    int fec_recovered_thresh_low;
    int probing_stability_ticks;
    int probing_step_ticks;
    int max_probe_success_count;

    int power_level; // 0 to 4
    int raw_pwr_matrix[5][8];
    int snr_mcs[8];

    // Prediction Configs
    int predict_time_s;
    int predict_delta;
    int predict_low_mcs_max;
    int predict_high_mcs_min;
    
    int osd_level;
    int osd_font_size;
    int osd_colour;
} QlinkConfig;

QlinkConfig cfg;

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

typedef enum { MSG_TYPE_INVALID = 0, MSG_TYPE_TELEMETRY, MSG_TYPE_SPECIAL } GsMessageType;

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
    int can_uplink; 
    int probe_success_count;
    int rawpower;
} LinkThreshold;

// Prediction State Machine
typedef enum {
    PREDICT_IDLE = 0,
    PREDICT_START_MAX,
    PREDICT_WAIT_MAX,
    PREDICT_START_MIN,
    PREDICT_WAIT_MIN,
    PREDICT_STEP_PWR,
    PREDICT_DONE
} PredictState;

typedef struct {
    PredictState state;
    int ticks_counter;
    long long sum_nf;
    int sample_count;
    int max_pwr_nf;
    int baseline_nf;
    int test_pwr;
    int max_safe_close_pwr;
    char status_msg[64];
} PredictEngine;

// Shared State struct for safe multithreading
typedef struct {
    GsTelemetry telemetry;
    downlink dl;
    bool has_new_data;
    int uplink_stable_count;   
    int cooldown_ticks;
    int no_telemetry_ticks;
    
    bool legacy_mode;
    bool failsafe_mode;
    bool log_corrupted;
    bool log_inconsistent;
    time_t corruption_time;
    LinkThreshold display_th[8]; 
    
    bool is_probing;
    int probe_target_mcs;
    int probe_original_mcs;
    int probe_step_ticks;
    int probe_failed_count[8];
    int probe_wait_ticks;
    
    PredictEngine predict;
    
    osd_udp_config_t osd_config;
    pthread_mutex_t lock;
} SharedData;

// --- Config Management ---

void create_default_config() {
    FILE *f = fopen(CONFIG_FILE_PATH, "w");
    if (!f) return;

    fprintf(f, "# QLink Auto-Generated Configuration\n\n");
    fprintf(f, "OVERHEAD_RATIO=0.25\nSAFETY_MARGIN=0.60\nOFFSET_EVM1=2\nOFFSET_EVM2=2\n");
    fprintf(f, "OFFSET_RSSI1=2\nOFFSET_SNR1=2\nUPLINK_STABILITY_TICKS=60\nCHANGE_COOLDOWN_TICKS=60\n");
    fprintf(f, "CMD_DELAY_US=9000\nNO_TELEMETRY_TICKS_THRESH=40\nDOWNLINK_LOST_PKTS_THRESH=2\n");
    fprintf(f, "DOWNLINK_LOST_PKTS67_THRESH=6\nDOWNLINK_LOST_PKTS67_C_THRESH=3\nUPLINK_LOST_PKTS_THRESH=0\n");
    fprintf(f, "FEC_N_CONSTANT=12\nFEC_K_PROBING_START=5\nFEC_K_HIGH_PROTECTION=8\nFEC_K_MED_PROTECTION=9\n");
    fprintf(f, "FEC_K_LOW_PROTECTION=10\nFEC_K_FAILSAFE=2\nFEC_RECOVERED_THRESH_HIGH=3\nFEC_RECOVERED_THRESH_LOW=1\n");
    fprintf(f, "PROBING_STABILITY_TICKS=70\nPROBING_STEP_TICKS=10\nMAX_PROBE_SUCCESS_COUNT=10\n\n");
    
    fprintf(f, "# Target SNR per MCS (0 to 7)\n");
    fprintf(f, "SNR_TARGETS=8,10,13,16,20,24,28,32\n\n");

    fprintf(f, "# Power Level Selector (0 = Min, 4 = Max)\n");
    fprintf(f, "POWER_LEVEL=4\n\n");

    fprintf(f, "# Power Matrices (MCS0 to MCS7)\n");
    fprintf(f, "PWR_L0=100,100,100,100,100,100,100,100\n");
    fprintf(f, "PWR_L1=1000,800,600,400,200,100,100,100\n");
    fprintf(f, "PWR_L2=1500,1500,1500,1500,1500,1500,1500,1500\n");
    fprintf(f, "PWR_L3=2500,2500,2250,2000,1750,1750,1500,1250\n");
    fprintf(f, "PWR_L4=2900,2750,2500,2250,1900,1900,1900,1900\n\n");

    fprintf(f, "# Prediction Routine Settings\n");
    fprintf(f, "PREDICT_TIME_S=20\nPREDICT_DELTA=6\n");
    fprintf(f, "PREDICT_LOW_MCS_MAX=5\nPREDICT_HIGH_MCS_MIN=6\n\n");

    fprintf(f, "OSD_LEVEL=4\nOSD_FONT_SIZE=20\nOSD_COLOUR=7\n");
    fclose(f);
}

void load_config() {
    FILE *f = fopen(CONFIG_FILE_PATH, "r");
    if (!f) {
        printf(">> Config missing. Generating default: %s\n", CONFIG_FILE_PATH);
        create_default_config();
        f = fopen(CONFIG_FILE_PATH, "r");
    }
    
    // Default fail-safes if parsing fails
    cfg.power_level = 4;
    cfg.predict_time_s = 20;
    cfg.predict_delta = 6;
    cfg.predict_low_mcs_max = 5;
    cfg.predict_high_mcs_min = 6;
    
    char line[256];
    while(fgets(line, sizeof(line), f)) {
        if(line[0] == '#' || line[0] == '\n') continue;
        
        sscanf(line, "OVERHEAD_RATIO=%f", &cfg.overhead_ratio);
        sscanf(line, "SAFETY_MARGIN=%f", &cfg.safety_margin);
        sscanf(line, "OFFSET_EVM1=%d", &cfg.offset_evm1);
        sscanf(line, "OFFSET_EVM2=%d", &cfg.offset_evm2);
        sscanf(line, "OFFSET_RSSI1=%d", &cfg.offset_rssi1);
        sscanf(line, "OFFSET_SNR1=%d", &cfg.offset_snr1);
        sscanf(line, "UPLINK_STABILITY_TICKS=%d", &cfg.uplink_stability_ticks);
        sscanf(line, "CHANGE_COOLDOWN_TICKS=%d", &cfg.change_cooldown_ticks);
        sscanf(line, "CMD_DELAY_US=%d", &cfg.cmd_delay_us);
        sscanf(line, "NO_TELEMETRY_TICKS_THRESH=%d", &cfg.no_telemetry_ticks_thresh);
        sscanf(line, "DOWNLINK_LOST_PKTS_THRESH=%d", &cfg.downlink_lost_pkts_thresh);
        sscanf(line, "DOWNLINK_LOST_PKTS67_THRESH=%d", &cfg.downlink_lost_pkts67_thresh);
        sscanf(line, "DOWNLINK_LOST_PKTS67_C_THRESH=%d", &cfg.downlink_lost_pkts67_c_thresh);
        sscanf(line, "UPLINK_LOST_PKTS_THRESH=%d", &cfg.uplink_lost_pkts_thresh);
        sscanf(line, "FEC_N_CONSTANT=%d", &cfg.fec_n_constant);
        sscanf(line, "FEC_K_PROBING_START=%d", &cfg.fec_k_probing_start);
        sscanf(line, "FEC_K_HIGH_PROTECTION=%d", &cfg.fec_k_high_protection);
        sscanf(line, "FEC_K_MED_PROTECTION=%d", &cfg.fec_k_med_protection);
        sscanf(line, "FEC_K_LOW_PROTECTION=%d", &cfg.fec_k_low_protection);
        sscanf(line, "FEC_K_FAILSAFE=%d", &cfg.fec_k_failsafe);
        sscanf(line, "FEC_RECOVERED_THRESH_HIGH=%d", &cfg.fec_recovered_thresh_high);
        sscanf(line, "FEC_RECOVERED_THRESH_LOW=%d", &cfg.fec_recovered_thresh_low);
        sscanf(line, "PROBING_STABILITY_TICKS=%d", &cfg.probing_stability_ticks);
        sscanf(line, "PROBING_STEP_TICKS=%d", &cfg.probing_step_ticks);
        sscanf(line, "MAX_PROBE_SUCCESS_COUNT=%d", &cfg.max_probe_success_count);
        sscanf(line, "POWER_LEVEL=%d", &cfg.power_level);
        sscanf(line, "PREDICT_TIME_S=%d", &cfg.predict_time_s);
        sscanf(line, "PREDICT_DELTA=%d", &cfg.predict_delta);
        sscanf(line, "PREDICT_LOW_MCS_MAX=%d", &cfg.predict_low_mcs_max);
        sscanf(line, "PREDICT_HIGH_MCS_MIN=%d", &cfg.predict_high_mcs_min);
        sscanf(line, "OSD_LEVEL=%d", &cfg.osd_level);
        sscanf(line, "OSD_FONT_SIZE=%d", &cfg.osd_font_size);
        sscanf(line, "OSD_COLOUR=%d", &cfg.osd_colour);

        if(strncmp(line, "SNR_TARGETS=", 12) == 0) {
            sscanf(line+12, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.snr_mcs[0], &cfg.snr_mcs[1], &cfg.snr_mcs[2], &cfg.snr_mcs[3], &cfg.snr_mcs[4], &cfg.snr_mcs[5], &cfg.snr_mcs[6], &cfg.snr_mcs[7]);
        }
        if(strncmp(line, "PWR_L0=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[0][0], &cfg.raw_pwr_matrix[0][1], &cfg.raw_pwr_matrix[0][2], &cfg.raw_pwr_matrix[0][3], &cfg.raw_pwr_matrix[0][4], &cfg.raw_pwr_matrix[0][5], &cfg.raw_pwr_matrix[0][6], &cfg.raw_pwr_matrix[0][7]);
        if(strncmp(line, "PWR_L1=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[1][0], &cfg.raw_pwr_matrix[1][1], &cfg.raw_pwr_matrix[1][2], &cfg.raw_pwr_matrix[1][3], &cfg.raw_pwr_matrix[1][4], &cfg.raw_pwr_matrix[1][5], &cfg.raw_pwr_matrix[1][6], &cfg.raw_pwr_matrix[1][7]);
        if(strncmp(line, "PWR_L2=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[2][0], &cfg.raw_pwr_matrix[2][1], &cfg.raw_pwr_matrix[2][2], &cfg.raw_pwr_matrix[2][3], &cfg.raw_pwr_matrix[2][4], &cfg.raw_pwr_matrix[2][5], &cfg.raw_pwr_matrix[2][6], &cfg.raw_pwr_matrix[2][7]);
        if(strncmp(line, "PWR_L3=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[3][0], &cfg.raw_pwr_matrix[3][1], &cfg.raw_pwr_matrix[3][2], &cfg.raw_pwr_matrix[3][3], &cfg.raw_pwr_matrix[3][4], &cfg.raw_pwr_matrix[3][5], &cfg.raw_pwr_matrix[3][6], &cfg.raw_pwr_matrix[3][7]);
        if(strncmp(line, "PWR_L4=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[4][0], &cfg.raw_pwr_matrix[4][1], &cfg.raw_pwr_matrix[4][2], &cfg.raw_pwr_matrix[4][3], &cfg.raw_pwr_matrix[4][4], &cfg.raw_pwr_matrix[4][5], &cfg.raw_pwr_matrix[4][6], &cfg.raw_pwr_matrix[4][7]);
    }
    fclose(f);
}

// --- 2. Link Math & Validation ---

const float BASE_RATES[5][2][8] = {
    { {1.5,  3.0,  4.5,  6.0,  9.0, 12.0, 13.5, 15.0}, {1.7,  3.4,  5.1,  6.8, 10.2, 13.6, 15.0, 16.7} },
    { {3.0,  6.0,  9.0, 12.0, 18.0, 24.0, 27.0, 30.0}, {3.3,  6.8, 10.2, 13.6, 20.4, 27.2, 30.0, 33.4} },
    { {6.5, 13.0, 19.5, 26.0, 39.0, 52.0, 58.5, 65.0}, {7.2, 14.4, 21.7, 28.9, 43.3, 57.8, 65.0, 72.2} },
    { {13.5, 27.0, 40.5, 54.0, 81.0, 108.0, 121.5, 135.0}, {15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0} },
    { {27.0, 54.0, 81.0, 108.0, 162.0, 216.0, 243.0, 270.0}, {30.0, 60.0, 90.0, 120.0, 180.0, 240.0, 270.0, 300.0} }
};

bool is_stricter(LinkThreshold lower, LinkThreshold higher, bool legacy) {
    if (!lower.valid || !higher.valid) return false;
    if (lower.rssi1 != 0 && higher.rssi1 != 0 && lower.rssi1 > higher.rssi1) return true;
    if (lower.snr1 != 0 && higher.snr1 != 0 && lower.snr1 > higher.snr1) return true;
    if (!legacy) {
        if (lower.evm1 != 0 && higher.evm1 != 0 && lower.evm1 < higher.evm1) return true;
        if (lower.evm2 != 0 && higher.evm2 != 0 && lower.evm2 < higher.evm2) return true;
    }
    return false;
}

bool is_identical(LinkThreshold a, LinkThreshold b, bool legacy) {
    if (!a.valid || !b.valid) return false;
    if (a.rssi1 == b.rssi1 && a.snr1 == b.snr1) {
        if (legacy) return true;
        if (a.evm1 == b.evm1 && a.evm2 == b.evm2) return true;
    }
    return false;
}

void apply_link_settings(const downlink* d, int raw_power_override) {
    static downlink prev = { .mcs = -1, .bitrate = -1, .feck = -1, .fecn = -1, .bw = -1, .gi = -1 };
    char cmd[256];
    
    int mcs_safe = (d->mcs >= 0 && d->mcs <= 7) ? d->mcs : 0;
    int lvl_safe = (cfg.power_level >= 0 && cfg.power_level <= 4) ? cfg.power_level : 4;
    int pwr_to_apply = (raw_power_override != -1) ? raw_power_override : cfg.raw_pwr_matrix[lvl_safe][mcs_safe];

    bool is_uplink = (prev.mcs != -1) && (d->mcs > prev.mcs);

    if (is_uplink) {
        snprintf(cmd, sizeof(cmd), "iw dev wlan0 set txpower fixed %d", pwr_to_apply);
        system(cmd); usleep(cfg.cmd_delay_us);
        
        if (d->mcs != prev.mcs || d->bw != prev.bw || d->gi != prev.gi) {
            const char* gi_str = (d->gi == 1) ? "short" : "long";
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_radio -B %d -G %s -S 1 -L 1 -M %d", d->bw, gi_str, d->mcs);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        if (d->feck != prev.feck || d->fecn != prev.fecn) {
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_fec -k %d -n %d", d->feck, d->fecn);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        if (d->bitrate != prev.bitrate) {
            snprintf(cmd, sizeof(cmd), "curl -s 'http://127.0.0.1/api/v1/set?video0.bitrate=%d'", d->bitrate);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
    } else {
        if (d->bitrate != prev.bitrate) {
            snprintf(cmd, sizeof(cmd), "curl -s 'http://127.0.0.1/api/v1/set?video0.bitrate=%d'", d->bitrate);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        if (d->feck != prev.feck || d->fecn != prev.fecn) {
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_fec -k %d -n %d", d->feck, d->fecn);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        if (d->mcs != prev.mcs || d->bw != prev.bw || d->gi != prev.gi) {
            const char* gi_str = (d->gi == 1) ? "short" : "long";
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_radio -B %d -G %s -S 1 -L 1 -M %d", d->bw, gi_str, d->mcs);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        snprintf(cmd, sizeof(cmd), "iw dev wlan0 set txpower fixed %d", pwr_to_apply);
        system(cmd); usleep(cfg.cmd_delay_us);
    }

    prev = *d;
    printf(">> [SYSTEM] Applied -> MCS:%d | BW:%d | GI:%s | FEC:%d/%d | Bitrate:%dkbps | Pwr:%d\n",
           d->mcs, d->bw, (d->gi == 1) ? "short" : "long", d->feck, d->fecn, d->bitrate, pwr_to_apply);
}

int calculate_safe_bitrate(int mcs, int bw, int fec_k, int fec_n, int gi) {
    if (mcs < 0 || mcs > 7) return 0;
    if (gi < 0 || gi > 1) return 0;
    int bw_idx = -1;
    switch(bw) {
        case 5:  bw_idx = 0; break; case 10: bw_idx = 1; break;
        case 20: bw_idx = 2; break; case 40: bw_idx = 3; break; case 80: bw_idx = 4; break;
        default: return 0;
    }
    float base_rate = BASE_RATES[bw_idx][gi][mcs];
    float usable = base_rate - (base_rate * cfg.overhead_ratio);
    float fec_overhead_ratio = (fec_n > 0 && fec_k <= fec_n) ? (float)(fec_n - fec_k) / (float)fec_n : 0.0f;
    float max_app_mbps = usable - (usable * fec_overhead_ratio);
    return (int)((max_app_mbps * cfg.safety_margin) * 1000.0f);
}

void update_downlink_bitrate(downlink* d) {
    d->bitrate = calculate_safe_bitrate(d->mcs, d->bw, d->feck, d->fecn, d->gi);
}

void downlink_init(downlink* d) {
    d->mcs = 7;  
    d->feck = cfg.fec_k_med_protection;
    d->fecn = cfg.fec_n_constant;
    d->bw = 20; d->roi = 0; d->gi = 0;
    update_downlink_bitrate(d);
    apply_link_settings(d, -1);
}

// --- 3. Parser & Logic ---

bool parse_telemetry_string(const char *msg, GsTelemetry *telemetry) {
    if (!msg || !telemetry) return false;
    memset(telemetry, 0, sizeof(GsTelemetry));
    telemetry->evm1 = 0; telemetry->evm2 = 0; telemetry->rssi1 = -105;

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
        if (cfg.osd_level == 0) continue;

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
        int noise_floor = rssi - snr;
        
        bool legacy = shared->legacy_mode;
        bool failsafe = shared->failsafe_mode;
        bool corrupted = shared->log_corrupted;
        bool inconsistent = shared->log_inconsistent;
        bool is_probing = shared->is_probing;
        bool is_predicting = (shared->predict.state != PREDICT_IDLE);
        char p_msg[64]; strcpy(p_msg, shared->predict.status_msg);
        
        LinkThreshold local_th[8];
        memcpy(local_th, shared->display_th, sizeof(local_th));
        pthread_mutex_unlock(&shared->lock);

        char rdy_str[16];
        if (is_predicting) strcpy(rdy_str, "CAL");
        else if (is_probing) strcpy(rdy_str, "PRB");
        else if (cooldown > 0) strcpy(rdy_str, "WAIT");
        else strcpy(rdy_str, "RDY");

        char mode_str[128];
        char err_str[64] = "";
        if (is_predicting) strcpy(err_str, p_msg);
        else if (corrupted) strcpy(err_str, "[CORRUPTED LOG]");
        else if (inconsistent) strcpy(err_str, "[LOG INCONSISTENCY]");

        snprintf(mode_str, sizeof(mode_str), "Modes: %s / %s %s", 
            legacy ? "LEGACY" : "STANDARD",
            failsafe ? "FAILSAFE" : "NORMAL", err_str);

        char log_str[1024] = "LOG [M:V:E1:E2:R:S:U:P:PWR]\n";
        for (int i = 7; i >= 0; i--) {
            char line[64];
            if (local_th[i].valid) {
                snprintf(line, sizeof(line), "%d:1:%d:%d:%d:%d:%d:%d:%d\n", 
                    i, local_th[i].evm1, local_th[i].evm2, local_th[i].rssi1, local_th[i].snr1, local_th[i].can_uplink, local_th[i].probe_success_count, local_th[i].rawpower);
            } else {
                snprintf(line, sizeof(line), "%d:0:0:0:0:0:0:0:0\n", i);
            }
            strcat(log_str, line);
        }

        char full_osd_string[2048];
        snprintf(full_osd_string, sizeof(full_osd_string), 
            "&L%d0&F%d MCS:%d FEC:%d/%d EVM:%d RSSI:%d SNR:%d NF:%d LST:%d REC:%d [%s]\n%s\n%s",
            cfg.osd_colour, cfg.osd_font_size, mcs, feck, fecn, evm, rssi, snr, noise_floor, lost, rec, rdy_str, mode_str, log_str);

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

// --- Pseudo-File Command Listener ---
void* fifo_listener_thread(void* arg) {
    SharedData *shared = (SharedData *)arg;
    mkfifo(FIFO_PATH, 0666);
    char buf[128];

    while (1) {
        int fd = open(FIFO_PATH, O_RDONLY);
        if (fd < 0) { sleep(1); continue; }
        
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            if (buf[n-1] == '\n') buf[n-1] = '\0'; // trim newline
            
            pthread_mutex_lock(&shared->lock);
            if (strncmp(buf, "delete_log", 10) == 0) {
                unlink(LOG_FILE_PATH);
                for(int k=0; k<=7; k++) shared->display_th[k].valid = false;
                shared->failsafe_mode = true;
                shared->log_corrupted = true;
                shared->corruption_time = time(NULL);
                printf(">> [CMD] Log deleted. Failsafe activated.\n");
            } 
            else if (strncmp(buf, "predict", 7) == 0) {
                if (shared->predict.state == PREDICT_IDLE) {
                    shared->predict.state = PREDICT_START_MAX;
                    printf(">> [CMD] Calibration (Prediction) Routine Started.\n");
                }
            }
            else if (strncmp(buf, "set_powerlevel", 14) == 0) {
                int plvl;
                if (sscanf(buf + 14, "%d", &plvl) == 1 && plvl >= 0 && plvl <= 4) {
                    cfg.power_level = plvl;
                    printf(">> [CMD] Power level set to %d\n", plvl);
                }
            }
            pthread_mutex_unlock(&shared->lock);
        }
        close(fd);
    }
    return NULL;
}

void* udp_listener_thread(void* arg) {
    SharedData *shared = (SharedData *)arg;
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    uint8_t buffer[BUFFER_SIZE];

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { perror("Socket creation failed"); pthread_exit(NULL); }
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(LISTEN_PORT);

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) { perror("Bind failed"); close(sockfd); pthread_exit(NULL); }

    char special_cmd[256];
    GsTelemetry temp_telemetry;

    while (1) {
        int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&client_addr, &client_addr_len);
        if (n < 0) break;
        buffer[n] = '\0';
        GsMessageType msg_type = parse_gs_packet(buffer, n, &temp_telemetry, special_cmd, sizeof(special_cmd));

        switch (msg_type) {
            case MSG_TYPE_TELEMETRY:
                pthread_mutex_lock(&shared->lock);
                shared->telemetry = temp_telemetry;
                shared->has_new_data = true;
                if (temp_telemetry.evm1 > 45 || temp_telemetry.evm2 > 45) {
                    shared->legacy_mode = true;
                }
                pthread_mutex_unlock(&shared->lock);
                break;
            case MSG_TYPE_SPECIAL:
                if (strncmp(special_cmd, "special:request_keyframe", 24) == 0) system("curl -s 'http://127.0.0.1/request/idr'");
                break;
            default: break;
        }
    }
    close(sockfd);
    return NULL;
}

// --- 4. Log File Management ---

bool validate_logs(LinkThreshold *th, bool legacy_mode) {
    for (int i = 0; i < 7; i++) {
        if (!th[i].valid) continue;
        for (int j = i + 1; j <= 7; j++) {
            if (!th[j].valid) continue;
            if (is_identical(th[i], th[j], legacy_mode)) return false; 
            if (is_stricter(th[i], th[j], legacy_mode)) return false; 
        }
    }
    return true;
}

void load_logs(LinkThreshold *th, bool *legacy_mode) {
    for (int i = 0; i <= 7; i++) { th[i].valid = false; th[i].can_uplink = 0; th[i].probe_success_count = 0; th[i].rawpower = 0; }
    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (!f) return;

    char header[32];
    int legacy_val = 0;
    if (fscanf(f, "%s %d", header, &legacy_val) == 2 && strcmp(header, "LEGACY") == 0) *legacy_mode = (legacy_val == 1);
    else rewind(f); 

    int mcs, valid, evm1, evm2, rssi1, snr1, can_uplink, probe_succ, rawpwr;
    while (fscanf(f, "%d %d %d %d %d %d %d %d %d", &mcs, &valid, &evm1, &evm2, &rssi1, &snr1, &can_uplink, &probe_succ, &rawpwr) == 9) {
        if (mcs >= 0 && mcs <= 7) {
            th[mcs].valid = valid; th[mcs].evm1 = evm1; th[mcs].evm2 = evm2;
            th[mcs].rssi1 = rssi1; th[mcs].snr1 = snr1; th[mcs].can_uplink = can_uplink;
            th[mcs].probe_success_count = probe_succ; th[mcs].rawpower = rawpwr;
        }
    }
    fclose(f);
}

void save_logs(LinkThreshold *th, bool legacy_mode) {
    char temp_path[256];
    snprintf(temp_path, sizeof(temp_path), "%s.tmp", LOG_FILE_PATH);

    FILE *f = fopen(temp_path, "w");
    if (!f) return;

    fprintf(f, "LEGACY %d\n", legacy_mode ? 1 : 0);
    for (int i = 0; i <= 7; i++) {
        fprintf(f, "%d %d %d %d %d %d %d %d %d\n", i, th[i].valid, th[i].evm1, th[i].evm2, th[i].rssi1, th[i].snr1, th[i].can_uplink, th[i].probe_success_count, th[i].rawpower);
    }
    fflush(f); fsync(fileno(f)); fclose(f);
    rename(temp_path, LOG_FILE_PATH);
}

// --- Prediction Routine ---
bool handle_prediction(SharedData *state, LinkThreshold *th) {
    PredictEngine *pe = &state->predict;
    int required_ticks = cfg.predict_time_s * 40; // 40Hz loop
    
    switch (pe->state) {
        case PREDICT_START_MAX:
            strcpy(pe->status_msg, "[CAL: MAX PWR]");
            state->dl.mcs = 0;
            state->dl.feck = cfg.fec_k_med_protection;
            update_downlink_bitrate(&state->dl);
            apply_link_settings(&state->dl, cfg.raw_pwr_matrix[4][0]); // Max power
            pe->ticks_counter = 0;
            pe->sum_nf = 0;
            pe->sample_count = 0;
            pe->state = PREDICT_WAIT_MAX;
            return true;

        case PREDICT_WAIT_MAX:
            if (state->has_new_data && state->telemetry.rssi1 != 0 && state->telemetry.snr1 != 0) {
                pe->sum_nf += (state->telemetry.rssi1 - state->telemetry.snr1);
                pe->sample_count++;
            }
            pe->ticks_counter++;
            if (pe->ticks_counter >= required_ticks) {
                if (pe->sample_count > 0) pe->max_pwr_nf = pe->sum_nf / pe->sample_count;
                else pe->max_pwr_nf = -100;
                pe->state = PREDICT_START_MIN;
            }
            return true;

        case PREDICT_START_MIN:
            strcpy(pe->status_msg, "[CAL: MIN PWR]");
            apply_link_settings(&state->dl, cfg.raw_pwr_matrix[0][0]); // Min power
            pe->ticks_counter = 0;
            pe->sum_nf = 0;
            pe->sample_count = 0;
            pe->state = PREDICT_WAIT_MIN;
            return true;

        case PREDICT_WAIT_MIN:
            if (state->has_new_data && state->telemetry.rssi1 != 0 && state->telemetry.snr1 != 0) {
                pe->sum_nf += (state->telemetry.rssi1 - state->telemetry.snr1);
                pe->sample_count++;
            }
            pe->ticks_counter++;
            if (pe->ticks_counter >= required_ticks) {
                if (pe->sample_count > 0) pe->baseline_nf = pe->sum_nf / pe->sample_count;
                else pe->baseline_nf = -100;
                
                if (pe->baseline_nf > -75) {
                    printf(">> [PREDICT] Too noisy (Baseline: %d). Aborting.\n", pe->baseline_nf);
                    strcpy(pe->status_msg, "[CAL: TOO NOISY]");
                    pe->state = PREDICT_IDLE; // Abort
                    return true;
                }
                
                if (pe->max_pwr_nf - pe->baseline_nf > cfg.predict_delta) {
                    pe->test_pwr = cfg.raw_pwr_matrix[0][0]; // Start at minimum
                    pe->max_safe_close_pwr = pe->test_pwr;
                    pe->state = PREDICT_STEP_PWR;
                    strcpy(pe->status_msg, "[CAL: STEPPING]");
                    apply_link_settings(&state->dl, pe->test_pwr);
                    pe->ticks_counter = 0;
                } else {
                    pe->max_safe_close_pwr = cfg.raw_pwr_matrix[4][0]; // Max is safe
                    pe->state = PREDICT_DONE;
                }
            }
            return true;

        case PREDICT_STEP_PWR:
            pe->ticks_counter++;
            if (pe->ticks_counter >= 10) { // Every 10 ticks check
                if (state->has_new_data) {
                    int current_nf = state->telemetry.rssi1 - state->telemetry.snr1;
                    if (current_nf - pe->baseline_nf < cfg.predict_delta) {
                        // Safe, increase
                        pe->max_safe_close_pwr = pe->test_pwr;
                        pe->test_pwr += 100;
                        apply_link_settings(&state->dl, pe->test_pwr);
                        pe->ticks_counter = 0;
                    } else {
                        // Exceeded delta, stop here
                        pe->state = PREDICT_DONE;
                    }
                }
            }
            return true;

        case PREDICT_DONE:
            strcpy(pe->status_msg, "");
            printf(">> [PREDICT] Complete. Baseline NF: %d, Max Safe Pwr: %d\n", pe->baseline_nf, pe->max_safe_close_pwr);
            
            int nf_calc = pe->baseline_nf + 2;
            for (int i = 0; i <= 7; i++) {
                th[i].valid = true;
                th[i].can_uplink = true;
                th[i].probe_success_count = 0;
                th[i].evm1 = 0; th[i].evm2 = 0; // Don't enforce EVM artificially
                th[i].snr1 = cfg.snr_mcs[i];
                th[i].rssi1 = nf_calc + cfg.snr_mcs[i];
                
                int lvl_safe = (cfg.power_level >= 0 && cfg.power_level <= 4) ? cfg.power_level : 4;
                int base_pwr = cfg.raw_pwr_matrix[lvl_safe][i];
                
                if (i <= cfg.predict_low_mcs_max) {
                    th[i].rawpower = base_pwr;
                } else if (i >= cfg.predict_high_mcs_min) {
                    th[i].rawpower = (base_pwr > pe->max_safe_close_pwr) ? pe->max_safe_close_pwr : base_pwr;
                } else {
                    th[i].rawpower = base_pwr; // For any gaps
                }
            }
            
            // Force save and reset system
            save_logs(th, state->legacy_mode);
            state->log_corrupted = false;
            state->log_inconsistent = false;
            state->failsafe_mode = false;
            
            apply_link_settings(&state->dl, -1); // Restore normal
            pe->state = PREDICT_IDLE;
            return true;
            
        default: return false;
    }
}

// --- 7. Main Link Logic ---

bool g0ylink(const GsTelemetry* t, downlink* d, LinkThreshold* th, SharedData* state) {
    if (state->predict.state != PREDICT_IDLE) {
        handle_prediction(state, th);
        return false; // Skip normal logic during calibration
    }

    bool log_saved = false;
    bool settings_changed = false;

    if (!state->log_corrupted || (time(NULL) - state->corruption_time > 5)) {
        memcpy(state->display_th, th, sizeof(LinkThreshold)*8);
        if (state->log_corrupted) state->log_corrupted = false; 
    }
    if (!state->log_inconsistent || (time(NULL) - state->corruption_time > 5)) {
        if (state->log_inconsistent) state->log_inconsistent = false; 
    }

    for(int i=0; i<7; i++) {
        for(int j=i+1; j<=7; j++) {
            if (is_identical(th[i], th[j], state->legacy_mode)) {
                unlink(LOG_FILE_PATH);
                for(int k=0; k<=7; k++) th[k].valid = false;
                state->failsafe_mode = true;
                state->log_corrupted = true;
                state->corruption_time = time(NULL);
                memcpy(state->display_th, th, sizeof(LinkThreshold)*8);
                return false; 
            }
        }
    }

    if (state->cooldown_ticks > 0) state->cooldown_ticks--;

    if (state->failsafe_mode) {
        if (d->mcs != 0 || d->feck != cfg.fec_k_failsafe) {
            d->mcs = 0; d->feck = cfg.fec_k_failsafe;
            update_downlink_bitrate(d); apply_link_settings(d, -1);
        }

        int target_mcs = -1;
        for (int i = 1; i <= 7; i++) {
            if (th[i].valid && th[i].can_uplink) { target_mcs = i; break; }
        }

        bool can_escape = false;
        if (target_mcs != -1) {
            bool evm1_ok  = state->legacy_mode || (t->evm1 == 0) || (t->evm1 < th[target_mcs].evm1);
            bool evm2_ok  = state->legacy_mode || (t->evm2 == 0) || (t->evm2 < th[target_mcs].evm2);
            bool rssi1_ok = (t->rssi1 == 0) || (t->rssi1 > th[target_mcs].rssi1);
            bool snr1_ok  = (t->snr1 == 0)  || (t->snr1 > th[target_mcs].snr1);
            bool lost_ok  = (t->lost_packets <= cfg.uplink_lost_pkts_thresh);
            if (evm1_ok && evm2_ok && rssi1_ok && snr1_ok && lost_ok) can_escape = true;
        } else {
            if ((t->snr1 == 0 || t->snr1 > 25) && t->lost_packets <= cfg.uplink_lost_pkts_thresh) can_escape = true;
            target_mcs = 1; 
        }

        if (can_escape) {
            state->uplink_stable_count++;
            if (state->uplink_stable_count >= cfg.uplink_stability_ticks) {
                state->failsafe_mode = false;
                d->mcs = target_mcs;
                state->cooldown_ticks = cfg.change_cooldown_ticks;
                state->uplink_stable_count = 0;
                settings_changed = true;
            }
        } else { state->uplink_stable_count = 0; }
        
        if (settings_changed) { update_downlink_bitrate(d); apply_link_settings(d, (th[d->mcs].valid && th[d->mcs].rawpower > 0) ? th[d->mcs].rawpower : -1); }
        return false; 
    }

    if (state->is_probing) {
        if (t->lost_packets > 0) {
            state->is_probing = false;
            state->probe_failed_count[state->probe_target_mcs]++;
            d->mcs = state->probe_original_mcs;
            state->cooldown_ticks = cfg.change_cooldown_ticks;
            settings_changed = true;
            goto APPLY_CHANGES_BLOCK; 
        }

        if (state->cooldown_ticks == 0) {
            state->probe_step_ticks++;
            if (state->probe_step_ticks >= cfg.probing_step_ticks) {
                state->probe_step_ticks = 0;
                
                if (d->feck < cfg.fec_k_high_protection) {
                    d->feck++; 
                    settings_changed = true;
                    state->cooldown_ticks = cfg.change_cooldown_ticks;
                } else {
                    state->is_probing = false;
                    state->probe_failed_count[state->probe_target_mcs] = 0;
                    th[state->probe_target_mcs].probe_success_count++;
                    
                    LinkThreshold* tgt = &th[state->probe_target_mcs];
                    bool worse_signal = false;

                    if (!tgt->valid) {
                        worse_signal = true; 
                    } else {
                        if (t->rssi1 != 0 && tgt->rssi1 != 0 && t->rssi1 < tgt->rssi1) worse_signal = true;
                        if (t->snr1 != 0  && tgt->snr1 != 0  && t->snr1 < tgt->snr1) worse_signal = true;
                        if (!state->legacy_mode && t->evm1 != 0 && tgt->evm1 != 0 && t->evm1 > tgt->evm1) worse_signal = true;
                    }

                    if (worse_signal) {
                        tgt->valid = true;
                        tgt->evm1 = t->evm1 + cfg.offset_evm1;
                        tgt->evm2 = t->evm2 + cfg.offset_evm2;
                        tgt->rssi1 = t->rssi1 + cfg.offset_rssi1;
                        tgt->snr1 = t->snr1 + cfg.offset_snr1;
                        tgt->can_uplink = 1;
                        
                        if (tgt->rawpower == 0) {
                            int lvl_safe = (cfg.power_level >= 0 && cfg.power_level <= 4) ? cfg.power_level : 4;
                            tgt->rawpower = cfg.raw_pwr_matrix[lvl_safe][state->probe_target_mcs];
                        }

                        for (int j = state->probe_target_mcs + 1; j <= 7; j++) {
                            if (th[j].valid) {
                                if (tgt->rssi1 >= th[j].rssi1) tgt->rssi1 = th[j].rssi1 - 1;
                                if (tgt->snr1 >= th[j].snr1)   tgt->snr1 = th[j].snr1 - 1;
                                if (!state->legacy_mode) {
                                    if (tgt->evm1 <= th[j].evm1) tgt->evm1 = th[j].evm1 + 1;
                                    if (tgt->evm2 <= th[j].evm2) tgt->evm2 = th[j].evm2 + 1;
                                }
                            }
                        }

                        if (state->probe_original_mcs >= 0 && th[state->probe_original_mcs].valid) {
                            LinkThreshold* lwr = &th[state->probe_original_mcs];
                            bool lwr_too_conservative = false;
                            if (tgt->rssi1 < lwr->rssi1) lwr_too_conservative = true;
                            if (tgt->snr1 < lwr->snr1) lwr_too_conservative = true;
                            if (!state->legacy_mode && tgt->evm1 > lwr->evm1) lwr_too_conservative = true;
                            
                            if (lwr_too_conservative) { lwr->valid = false; lwr->can_uplink = 0; }
                        }
                        log_saved = true;
                    }
                }
            }
        }
        goto APPLY_CHANGES_BLOCK;
    }

    if (!state->is_probing && !state->failsafe_mode) {
        int target_feck = d->feck;
        if (t->recovered >= cfg.fec_recovered_thresh_high) target_feck = cfg.fec_k_high_protection; 
        else if (t->recovered <= cfg.fec_recovered_thresh_low) target_feck = cfg.fec_k_low_protection;  
        else target_feck = cfg.fec_k_med_protection;  

        if (target_feck != d->feck) { d->feck = target_feck; settings_changed = true; }
    }

    if (state->cooldown_ticks > 0) goto APPLY_CHANGES_BLOCK; 

    int old_mcs = d->mcs;
    bool trigger_downlink = false;
    bool can_uplink_now = false;
    
    // --- UPDATED PROBING TRIGGER ---
    if (d->mcs < 7 && t->lost_packets == 0 && t->recovered <= 1 && th[d->mcs+1].probe_success_count < cfg.max_probe_success_count) {
        int tgt_mcs = d->mcs + 1;
        int penalty = state->probe_failed_count[tgt_mcs];
        int threshold = cfg.snr_mcs[tgt_mcs] - 2 + penalty;
        
        if (t->snr1 != 0 && t->snr1 >= threshold) {
            state->probe_wait_ticks++;
            if (state->probe_wait_ticks >= cfg.probing_stability_ticks) {
                state->is_probing = true;
                state->probe_target_mcs = tgt_mcs;
                state->probe_original_mcs = d->mcs;
                state->probe_step_ticks = 0;
                
                d->mcs = tgt_mcs;
                d->feck = cfg.fec_k_probing_start;
                state->cooldown_ticks = cfg.change_cooldown_ticks;
                settings_changed = true;
                state->probe_wait_ticks = 0;
                goto APPLY_CHANGES_BLOCK;
            }
        } else {
            state->probe_wait_ticks = 0;
        }
    } else {
        state->probe_wait_ticks = 0;
    }

    int current_lost_thresh = cfg.downlink_lost_pkts_thresh;
    if (old_mcs >= 6) {
        if (th[old_mcs].valid) current_lost_thresh = cfg.downlink_lost_pkts67_c_thresh;
        else current_lost_thresh = cfg.downlink_lost_pkts67_thresh;
    }

    if (th[old_mcs].valid) {
        for (int j = old_mcs + 1; j <= 7; j++) {
            if (is_stricter(th[old_mcs], th[j], state->legacy_mode)) {
                th[old_mcs].valid = false; th[old_mcs].can_uplink = 0;
                state->failsafe_mode = true; state->log_inconsistent = true;
                state->corruption_time = time(NULL);
                memcpy(state->display_th, th, sizeof(LinkThreshold)*8);
                return true; 
            }
        }
    }

    if (th[old_mcs].valid) {
        int bad_evm1 = th[old_mcs].evm1 - cfg.offset_evm1;
        int bad_evm2 = th[old_mcs].evm2 - cfg.offset_evm2;
        int bad_rssi1 = th[old_mcs].rssi1 - cfg.offset_rssi1;
        int bad_snr1 = th[old_mcs].snr1 - cfg.offset_snr1;

        bool evm1_bad  = !state->legacy_mode && (t->evm1 != 0)  && (t->evm1 >= bad_evm1);
        bool evm2_bad  = !state->legacy_mode && (t->evm2 != 0)  && (t->evm2 >= bad_evm2);
        bool rssi1_bad = (t->rssi1 != 0) && (t->rssi1 <= bad_rssi1);
        bool snr1_bad  = (t->snr1 != 0)  && (t->snr1 <= bad_snr1);
        bool lost_bad  = (t->lost_packets >= current_lost_thresh);

        if (evm1_bad || evm2_bad || rssi1_bad || snr1_bad || lost_bad) trigger_downlink = true;
    } else {
        if (t->lost_packets >= current_lost_thresh) trigger_downlink = true;
    }

    if (t->lost_packets > 0 && d->mcs == 0 && d->feck <= cfg.fec_k_high_protection) {
        state->failsafe_mode = true;
        return false;
    }

    if (trigger_downlink && d->mcs > 0) {
        LinkThreshold temp_th;
        temp_th.valid = true;
        temp_th.evm1 = t->evm1 + cfg.offset_evm1;
        temp_th.evm2 = t->evm2 + cfg.offset_evm2;
        temp_th.rssi1 = t->rssi1 + cfg.offset_rssi1;
        temp_th.snr1 = t->snr1 + cfg.offset_snr1;
        temp_th.can_uplink = 1;
        temp_th.probe_success_count = th[old_mcs].probe_success_count; 
        
        int lvl_safe = (cfg.power_level >= 0 && cfg.power_level <= 4) ? cfg.power_level : 4;
        temp_th.rawpower = th[old_mcs].rawpower > 0 ? th[old_mcs].rawpower : cfg.raw_pwr_matrix[lvl_safe][old_mcs];

        for (int j = old_mcs + 1; j <= 7; j++) {
            if (th[j].valid) {
                if (temp_th.rssi1 >= th[j].rssi1) temp_th.rssi1 = th[j].rssi1 - 1;
                if (temp_th.snr1 >= th[j].snr1)   temp_th.snr1 = th[j].snr1 - 1;
                if (!state->legacy_mode) {
                    if (temp_th.evm1 <= th[j].evm1) temp_th.evm1 = th[j].evm1 + 1;
                    if (temp_th.evm2 <= th[j].evm2) temp_th.evm2 = th[j].evm2 + 1;
                }
            }
        }

        th[old_mcs] = temp_th;
        log_saved = true;

        d->mcs--;
        state->uplink_stable_count = 0; 
        state->cooldown_ticks = cfg.change_cooldown_ticks; 
        settings_changed = true;
    } 
    else if (!trigger_downlink && d->mcs < 7) {
        int target_mcs = d->mcs + 1;
        bool conditions_met = false;

        for (int i = target_mcs; i <= 7; i++) {
            if (th[i].valid && th[i].can_uplink) {
                bool evm1_ok  = state->legacy_mode || (t->evm1 == 0)  || (t->evm1 < th[i].evm1);
                bool evm2_ok  = state->legacy_mode || (t->evm2 == 0)  || (t->evm2 < th[i].evm2);
                bool rssi1_ok = (t->rssi1 == 0) || (t->rssi1 > th[i].rssi1);
                bool snr1_ok  = (t->snr1 == 0)  || (t->snr1 > th[i].snr1);
                bool lost_ok  = (t->lost_packets <= cfg.uplink_lost_pkts_thresh);

                if (evm1_ok && evm2_ok && rssi1_ok && snr1_ok && lost_ok) {
                    conditions_met = true;
                    break;
                }
            }
        }

        if (conditions_met) {
            state->uplink_stable_count++;
            if (state->uplink_stable_count >= cfg.uplink_stability_ticks) {
                can_uplink_now = true;
                state->uplink_stable_count = 0; 
            }
        } else { state->uplink_stable_count = 0; }
    } else { state->uplink_stable_count = 0; }

    if (can_uplink_now) {
        d->mcs++;
        state->cooldown_ticks = cfg.change_cooldown_ticks;
        settings_changed = true;
    }

APPLY_CHANGES_BLOCK:
    if (settings_changed) {
        update_downlink_bitrate(d);
        apply_link_settings(d, (th[d->mcs].valid && th[d->mcs].rawpower > 0) ? th[d->mcs].rawpower : -1);
    }
    return log_saved;
}


// --- 8. Main ---

int main(void) {
    load_config(); // Read from /etc/qlink.conf

    SharedData shared;
    memset(&shared, 0, sizeof(shared));
    shared.has_new_data = false;
    shared.osd_config.udp_out_sock = -1; 
    
    downlink_init(&shared.dl);

    LinkThreshold thresholds[8];
    load_logs(thresholds, &shared.legacy_mode);

    if (!validate_logs(thresholds, shared.legacy_mode)) {
        unlink(LOG_FILE_PATH);
        for(int i=0; i<=7; i++) thresholds[i].valid = false;
        shared.failsafe_mode = true;
        shared.log_corrupted = true;
        shared.corruption_time = time(NULL);
        memcpy(shared.display_th, thresholds, sizeof(LinkThreshold)*8);
    }

    if (pthread_mutex_init(&shared.lock, NULL) != 0) return EXIT_FAILURE;

    pthread_t listener_tid, osd_tid, fifo_tid;
    pthread_create(&listener_tid, NULL, udp_listener_thread, &shared);
    pthread_create(&osd_tid, NULL, periodic_update_osd, &shared);
    pthread_create(&fifo_tid, NULL, fifo_listener_thread, &shared);

    while (1) {
        bool needs_save = false;
        pthread_mutex_lock(&shared.lock);
        
        if (!shared.has_new_data) {
            shared.no_telemetry_ticks++;
            if (shared.no_telemetry_ticks >= cfg.no_telemetry_ticks_thresh && !shared.failsafe_mode) shared.failsafe_mode = true;
        } else {
            shared.no_telemetry_ticks = 0;
        }

        needs_save = g0ylink(&shared.telemetry, &shared.dl, thresholds, &shared);
        shared.has_new_data = false;
        bool current_legacy_mode = shared.legacy_mode;
        pthread_mutex_unlock(&shared.lock);

        if (needs_save) save_logs(thresholds, current_legacy_mode);
        usleep(25000); 
    }

    return EXIT_SUCCESS;
}
