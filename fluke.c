/*
 * fluke.c — Adaptive Link Controller for OpenIPC FPV
 *
 * Integrates with:
 *   - waybeam_venc  (HTTP API on port 8888)
 *   - wfb-ng        (wfb_tx_cmd on port 8000)
 *   - iw            (txpower control)
 *
 * waybeam_venc API used:
 *   GET http://<VENC_IP>:<VENC_PORT>/api/v1/set?video0.bitrate=<kbps>
 *   GET http://<VENC_IP>:<VENC_PORT>/api/v1/set?video0.gop_size=<seconds>
 *   GET http://<VENC_IP>:<VENC_PORT>/api/v1/set?fpv.roi_qp=<delta>
 *   GET http://<VENC_IP>:<VENC_PORT>/request/idr
 *
 * All video0.bitrate and gop_size fields are 'live' — no pipeline restart needed.
 * fpv.roi_qp is 'live' as well.
 *
 * License: GPL-3.0
 */

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

#define LISTEN_PORT         9999
#define BUFFER_SIZE         2048
#define FIFO_PATH           "/tmp/qlink_cmd"
#define CONFIG_FILE_PATH    "/etc/fluke.conf"
#define LOG_FILE_PATH       "/etc/fluke.log"

/* waybeam_venc defaults — override via config */
#define VENC_DEFAULT_IP     "127.0.0.1"
#define VENC_DEFAULT_PORT   8888

/* ------------------------------------------------------------------ */
/* OSD UDP config                                                       */
/* ------------------------------------------------------------------ */
typedef struct {
    int  udp_out_sock;
    char udp_out_ip[INET_ADDRSTRLEN];
    int  udp_out_port;
} osd_udp_config_t;

/* ------------------------------------------------------------------ */
/* CONFIGURATION STRUCT                                                 */
/* ------------------------------------------------------------------ */
typedef struct {
    float overhead_ratio;
    float safety_margin;
    int   offset_evm1;
    int   offset_evm2;
    int   offset_rssi1;
    int   offset_snr1;
    int   uplink_stability_ticks;
    int   change_cooldown_ticks;
    int   cmd_delay_us;
    int   no_telemetry_ticks_thresh;
    int   downlink_lost_pkts_thresh;
    int   downlink_lost_pkts67_thresh;
    int   downlink_lost_pkts67_c_thresh;
    int   uplink_lost_pkts_thresh;
    int   fec_n_constant;
    int   fec_k_probing_start;
    int   fec_k_high_protection;
    int   fec_k_med_protection;
    int   fec_k_low_protection;
    int   fec_k_failsafe;
    int   fec_recovered_thresh_high;
    int   fec_recovered_thresh_low;
    int   probing_stability_ticks;
    int   probing_step_ticks;
    int   max_probe_success_count;
    int   power_level;           /* 0–4 */
    int   raw_pwr_matrix[5][8];
    int   snr_mcs[8];

    /* Prediction */
    int   predict_time_s;
    int   predict_delta;
    int   predict_low_mcs_max;
    int   predict_high_mcs_min;

    /* OSD */
    int   osd_level;
    int   osd_font_size;
    int   osd_colour;

    /* waybeam_venc */
    char  venc_ip[64];
    int   venc_port;

    /* GOP adaptation (seconds, float stored *100 as int for sscanf) */
    float gop_good_link;   /* GOP at clean high-MCS link  (e.g. 1.0 s) */
    float gop_bad_link;    /* GOP at degraded/low-MCS link (e.g. 0.25 s) */

    /* ROI QP adaptation */
    int   roi_qp_high_mcs; /* e.g. -4  (less aggressive, save bitrate) */
    int   roi_qp_low_mcs;  /* e.g. -18 (sharp center, tolerate blurry edges) */
    int   roi_mcs_threshold; /* MCS <= this → use roi_qp_low_mcs */
} QlinkConfig;

QlinkConfig cfg;

/* ------------------------------------------------------------------ */
/* Core structs / enums                                                 */
/* ------------------------------------------------------------------ */
typedef struct {
    int  transmitted_time;
    int  evm1;
    int  evm2;
    int  recovered;
    int  lost_packets;
    int  rssi1;
    int  snr1;
    int  num_antennas;
    int  noise_pnlty;
    int  fec_change;
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
    int  evm1;
    int  evm2;
    int  rssi1;
    int  snr1;
    int  can_uplink;
    int  probe_success_count;
    int  rawpower;
} LinkThreshold;

/* Prediction state machine */
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
    int       ticks_counter;
    long long sum_nf;
    int       sample_count;
    int       max_pwr_nf;
    int       baseline_nf;
    int       test_pwr;
    int       max_safe_close_pwr;
    char      status_msg[64];
} PredictEngine;

/* Shared state */
typedef struct {
    GsTelemetry   telemetry;
    downlink      dl;
    bool          has_new_data;
    int           uplink_stable_count;
    int           cooldown_ticks;
    int           no_telemetry_ticks;
    bool          legacy_mode;
    bool          failsafe_mode;
    bool          log_corrupted;
    bool          log_inconsistent;
    time_t        corruption_time;
    LinkThreshold display_th[8];
    bool          is_probing;
    int           probe_target_mcs;
    int           probe_original_mcs;
    int           probe_step_ticks;
    int           probe_failed_count[8];
    int           probe_wait_ticks;
    PredictEngine predict;
    osd_udp_config_t osd_config;

    /* waybeam_venc tracking — avoid redundant curl calls */
    int   last_venc_bitrate;
    float last_venc_gop;
    int   last_roi_qp;

    pthread_mutex_t lock;
} SharedData;

/* ------------------------------------------------------------------ */
/* Config Management                                                    */
/* ------------------------------------------------------------------ */
static void create_default_config(void) {
    FILE *f = fopen(CONFIG_FILE_PATH, "w");
    if (!f) return;
    fprintf(f, "# Fluke — Adaptive Link Controller\n");
    fprintf(f, "# Auto-generated defaults\n\n");
    fprintf(f, "OVERHEAD_RATIO=0.25\nSAFETY_MARGIN=0.40\n");
    fprintf(f, "OFFSET_EVM1=2\nOFFSET_EVM2=2\n");
    fprintf(f, "OFFSET_RSSI1=2\nOFFSET_SNR1=2\n");
    fprintf(f, "UPLINK_STABILITY_TICKS=60\nCHANGE_COOLDOWN_TICKS=60\n");
    fprintf(f, "CMD_DELAY_US=9000\nNO_TELEMETRY_TICKS_THRESH=40\n");
    fprintf(f, "DOWNLINK_LOST_PKTS_THRESH=2\n");
    fprintf(f, "DOWNLINK_LOST_PKTS67_THRESH=6\nDOWNLINK_LOST_PKTS67_C_THRESH=3\n");
    fprintf(f, "UPLINK_LOST_PKTS_THRESH=0\n");
    fprintf(f, "FEC_N_CONSTANT=12\nFEC_K_PROBING_START=5\n");
    fprintf(f, "FEC_K_HIGH_PROTECTION=8\nFEC_K_MED_PROTECTION=9\n");
    fprintf(f, "FEC_K_LOW_PROTECTION=10\nFEC_K_FAILSAFE=5\n");
    fprintf(f, "FEC_RECOVERED_THRESH_HIGH=3\nFEC_RECOVERED_THRESH_LOW=1\n");
    fprintf(f, "PROBING_STABILITY_TICKS=120\nPROBING_STEP_TICKS=10\n");
    fprintf(f, "MAX_PROBE_SUCCESS_COUNT=10\n\n");
    fprintf(f, "# Target SNR per MCS (0 to 7)\n");
    fprintf(f, "SNR_TARGETS=6,8,10,12,17,21,24,29\n\n");
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
    fprintf(f, "OSD_LEVEL=4\nOSD_FONT_SIZE=20\nOSD_COLOUR=7\n\n");
    fprintf(f, "# waybeam_venc connection\n");
    fprintf(f, "VENC_IP=127.0.0.1\nVENC_PORT=8888\n\n");
    fprintf(f, "# GOP adaptation (seconds)\n");
    fprintf(f, "# Good link: long GOP = efficient encoding\n");
    fprintf(f, "# Bad link:  short GOP = fast recovery after packet loss\n");
    fprintf(f, "GOP_GOOD_LINK=1.0\nGOP_BAD_LINK=0.25\n\n");
    fprintf(f, "# ROI QP adaptation\n");
    fprintf(f, "# Negative = sharper center (better for FPV piloting)\n");
    fprintf(f, "ROI_QP_HIGH_MCS=-4\nROI_QP_LOW_MCS=-18\n");
    fprintf(f, "ROI_MCS_THRESHOLD=2\n");
    fclose(f);
}

static void load_config(void) {
    FILE *f = fopen(CONFIG_FILE_PATH, "r");
    if (!f) {
        printf(">> Config missing. Generating default: %s\n", CONFIG_FILE_PATH);
        create_default_config();
        f = fopen(CONFIG_FILE_PATH, "r");
    }

    /* Defaults */
    cfg.power_level         = 4;
    cfg.predict_time_s      = 20;
    cfg.predict_delta       = 6;
    cfg.predict_low_mcs_max = 5;
    cfg.predict_high_mcs_min = 6;
    strncpy(cfg.venc_ip, VENC_DEFAULT_IP, sizeof(cfg.venc_ip) - 1);
    cfg.venc_port           = VENC_DEFAULT_PORT;
    cfg.gop_good_link       = 1.0f;
    cfg.gop_bad_link        = 0.25f;
    cfg.roi_qp_high_mcs     = -4;
    cfg.roi_qp_low_mcs      = -18;
    cfg.roi_mcs_threshold   = 2;

    if (!f) return;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n') continue;

        sscanf(line, "OVERHEAD_RATIO=%f",              &cfg.overhead_ratio);
        sscanf(line, "SAFETY_MARGIN=%f",               &cfg.safety_margin);
        sscanf(line, "OFFSET_EVM1=%d",                 &cfg.offset_evm1);
        sscanf(line, "OFFSET_EVM2=%d",                 &cfg.offset_evm2);
        sscanf(line, "OFFSET_RSSI1=%d",                &cfg.offset_rssi1);
        sscanf(line, "OFFSET_SNR1=%d",                 &cfg.offset_snr1);
        sscanf(line, "UPLINK_STABILITY_TICKS=%d",      &cfg.uplink_stability_ticks);
        sscanf(line, "CHANGE_COOLDOWN_TICKS=%d",       &cfg.change_cooldown_ticks);
        sscanf(line, "CMD_DELAY_US=%d",                &cfg.cmd_delay_us);
        sscanf(line, "NO_TELEMETRY_TICKS_THRESH=%d",   &cfg.no_telemetry_ticks_thresh);
        sscanf(line, "DOWNLINK_LOST_PKTS_THRESH=%d",   &cfg.downlink_lost_pkts_thresh);
        sscanf(line, "DOWNLINK_LOST_PKTS67_THRESH=%d", &cfg.downlink_lost_pkts67_thresh);
        sscanf(line, "DOWNLINK_LOST_PKTS67_C_THRESH=%d",&cfg.downlink_lost_pkts67_c_thresh);
        sscanf(line, "UPLINK_LOST_PKTS_THRESH=%d",     &cfg.uplink_lost_pkts_thresh);
        sscanf(line, "FEC_N_CONSTANT=%d",              &cfg.fec_n_constant);
        sscanf(line, "FEC_K_PROBING_START=%d",         &cfg.fec_k_probing_start);
        sscanf(line, "FEC_K_HIGH_PROTECTION=%d",       &cfg.fec_k_high_protection);
        sscanf(line, "FEC_K_MED_PROTECTION=%d",        &cfg.fec_k_med_protection);
        sscanf(line, "FEC_K_LOW_PROTECTION=%d",        &cfg.fec_k_low_protection);
        sscanf(line, "FEC_K_FAILSAFE=%d",              &cfg.fec_k_failsafe);
        sscanf(line, "FEC_RECOVERED_THRESH_HIGH=%d",   &cfg.fec_recovered_thresh_high);
        sscanf(line, "FEC_RECOVERED_THRESH_LOW=%d",    &cfg.fec_recovered_thresh_low);
        sscanf(line, "PROBING_STABILITY_TICKS=%d",     &cfg.probing_stability_ticks);
        sscanf(line, "PROBING_STEP_TICKS=%d",          &cfg.probing_step_ticks);
        sscanf(line, "MAX_PROBE_SUCCESS_COUNT=%d",     &cfg.max_probe_success_count);
        sscanf(line, "POWER_LEVEL=%d",                 &cfg.power_level);
        sscanf(line, "PREDICT_TIME_S=%d",              &cfg.predict_time_s);
        sscanf(line, "PREDICT_DELTA=%d",               &cfg.predict_delta);
        sscanf(line, "PREDICT_LOW_MCS_MAX=%d",         &cfg.predict_low_mcs_max);
        sscanf(line, "PREDICT_HIGH_MCS_MIN=%d",        &cfg.predict_high_mcs_min);
        sscanf(line, "OSD_LEVEL=%d",                   &cfg.osd_level);
        sscanf(line, "OSD_FONT_SIZE=%d",               &cfg.osd_font_size);
        sscanf(line, "OSD_COLOUR=%d",                  &cfg.osd_colour);
        sscanf(line, "VENC_PORT=%d",                   &cfg.venc_port);
        sscanf(line, "GOP_GOOD_LINK=%f",               &cfg.gop_good_link);
        sscanf(line, "GOP_BAD_LINK=%f",                &cfg.gop_bad_link);
        sscanf(line, "ROI_QP_HIGH_MCS=%d",             &cfg.roi_qp_high_mcs);
        sscanf(line, "ROI_QP_LOW_MCS=%d",              &cfg.roi_qp_low_mcs);
        sscanf(line, "ROI_MCS_THRESHOLD=%d",           &cfg.roi_mcs_threshold);

        if (strncmp(line, "VENC_IP=", 8) == 0) {
            sscanf(line + 8, "%63s", cfg.venc_ip);
            /* strip newline */
            char *nl = strchr(cfg.venc_ip, '\n');
            if (nl) *nl = '\0';
        }

        if (strncmp(line, "SNR_TARGETS=", 12) == 0) {
            sscanf(line + 12, "%d,%d,%d,%d,%d,%d,%d,%d",
                &cfg.snr_mcs[0], &cfg.snr_mcs[1], &cfg.snr_mcs[2], &cfg.snr_mcs[3],
                &cfg.snr_mcs[4], &cfg.snr_mcs[5], &cfg.snr_mcs[6], &cfg.snr_mcs[7]);
        }
        if (strncmp(line, "PWR_L0=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[0][0], &cfg.raw_pwr_matrix[0][1], &cfg.raw_pwr_matrix[0][2], &cfg.raw_pwr_matrix[0][3], &cfg.raw_pwr_matrix[0][4], &cfg.raw_pwr_matrix[0][5], &cfg.raw_pwr_matrix[0][6], &cfg.raw_pwr_matrix[0][7]);
        if (strncmp(line, "PWR_L1=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[1][0], &cfg.raw_pwr_matrix[1][1], &cfg.raw_pwr_matrix[1][2], &cfg.raw_pwr_matrix[1][3], &cfg.raw_pwr_matrix[1][4], &cfg.raw_pwr_matrix[1][5], &cfg.raw_pwr_matrix[1][6], &cfg.raw_pwr_matrix[1][7]);
        if (strncmp(line, "PWR_L2=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[2][0], &cfg.raw_pwr_matrix[2][1], &cfg.raw_pwr_matrix[2][2], &cfg.raw_pwr_matrix[2][3], &cfg.raw_pwr_matrix[2][4], &cfg.raw_pwr_matrix[2][5], &cfg.raw_pwr_matrix[2][6], &cfg.raw_pwr_matrix[2][7]);
        if (strncmp(line, "PWR_L3=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[3][0], &cfg.raw_pwr_matrix[3][1], &cfg.raw_pwr_matrix[3][2], &cfg.raw_pwr_matrix[3][3], &cfg.raw_pwr_matrix[3][4], &cfg.raw_pwr_matrix[3][5], &cfg.raw_pwr_matrix[3][6], &cfg.raw_pwr_matrix[3][7]);
        if (strncmp(line, "PWR_L4=", 7) == 0) sscanf(line+7, "%d,%d,%d,%d,%d,%d,%d,%d", &cfg.raw_pwr_matrix[4][0], &cfg.raw_pwr_matrix[4][1], &cfg.raw_pwr_matrix[4][2], &cfg.raw_pwr_matrix[4][3], &cfg.raw_pwr_matrix[4][4], &cfg.raw_pwr_matrix[4][5], &cfg.raw_pwr_matrix[4][6], &cfg.raw_pwr_matrix[4][7]);
    }
    fclose(f);

    printf(">> [CONFIG] venc at http://%s:%d  GOP good=%.2fs bad=%.2fs  ROI high=%d low=%d thr=%d\n",
        cfg.venc_ip, cfg.venc_port,
        cfg.gop_good_link, cfg.gop_bad_link,
        cfg.roi_qp_high_mcs, cfg.roi_qp_low_mcs, cfg.roi_mcs_threshold);
}

/* ------------------------------------------------------------------ */
/* waybeam_venc HTTP helpers                                            */
/* ------------------------------------------------------------------ */

/*
 * Set video0.bitrate on waybeam_venc.
 * Only issues curl when value has actually changed.
 */
static void venc_set_bitrate(SharedData *state, int kbps) {
    if (state->last_venc_bitrate == kbps) return;
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
        "curl -sf 'http://%s:%d/api/v1/set?video0.bitrate=%d' > /dev/null 2>&1",
        cfg.venc_ip, cfg.venc_port, kbps);
    system(cmd);
    state->last_venc_bitrate = kbps;
    printf(">> [VENC] bitrate → %d kbps\n", kbps);
}

/*
 * Set video0.gop_size on waybeam_venc.
 * gop_size is in seconds (float). Only updates on meaningful change (>0.05s).
 */
static void venc_set_gop(SharedData *state, float gop_s) {
    if (fabsf(state->last_venc_gop - gop_s) < 0.05f) return;
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
        "curl -sf 'http://%s:%d/api/v1/set?video0.gop_size=%.2f' > /dev/null 2>&1",
        cfg.venc_ip, cfg.venc_port, (double)gop_s);
    system(cmd);
    state->last_venc_gop = gop_s;
    printf(">> [VENC] gop_size → %.2fs\n", (double)gop_s);
}

/*
 * Set fpv.roi_qp on waybeam_venc.
 * Negative value = sharper center (recommended for low-MCS FPV).
 */
static void venc_set_roi_qp(SharedData *state, int qp) {
    if (state->last_roi_qp == qp) return;
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
        "curl -sf 'http://%s:%d/api/v1/set?fpv.roi_qp=%d' > /dev/null 2>&1",
        cfg.venc_ip, cfg.venc_port, qp);
    system(cmd);
    state->last_roi_qp = qp;
    printf(">> [VENC] roi_qp → %d\n", qp);
}

/*
 * Request an IDR keyframe from waybeam_venc.
 * Used after MCS downgrade to aid decoder resync.
 */
static void venc_request_idr(void) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
        "curl -sf 'http://%s:%d/request/idr' > /dev/null 2>&1",
        cfg.venc_ip, cfg.venc_port);
    system(cmd);
    printf(">> [VENC] IDR requested\n");
}

/*
 * Apply all venc parameters that depend on the current MCS/link quality.
 * Called whenever dl changes or after link assessment.
 *
 * Strategy:
 *  - bitrate: computed safe bitrate (same as before)
 *  - gop_size: long at good links (efficient), short at bad links (resilient)
 *  - roi_qp: gentle at high MCS, aggressive sharpening at low MCS
 */
static void venc_apply_for_mcs(SharedData *state, const downlink *d,
                                int lost_pkts, int recovered) {
    /* bitrate */
    venc_set_bitrate(state, d->bitrate);

    /* gop: if we have losses or at low MCS → short GOP for recovery */
    bool link_stressed = (lost_pkts > 0) || (recovered > 1) ||
                         (d->mcs <= cfg.roi_mcs_threshold);
    float gop = link_stressed ? cfg.gop_bad_link : cfg.gop_good_link;
    venc_set_gop(state, gop);

    /* roi_qp: sharpen center when link is marginal */
    int qp = (d->mcs <= cfg.roi_mcs_threshold)
              ? cfg.roi_qp_low_mcs
              : cfg.roi_qp_high_mcs;
    venc_set_roi_qp(state, qp);
}

/* ------------------------------------------------------------------ */
/* Link Math                                                            */
/* ------------------------------------------------------------------ */
static const float BASE_RATES[5][2][8] = {
    { {1.5f,3.0f,4.7f,6.6f,9.9f,14.4f,17.6f,15.0f},   {1.7f,3.4f,5.4f,7.5f,11.2f,16.3f,19.5f,16.7f} },
    { {3.0f,6.0f,9.5f,13.2f,19.8f,28.8f,35.1f,30.0f},  {3.3f,6.8f,10.7f,15.0f,22.4f,32.6f,39.0f,33.4f} },
    { {6.5f,13.0f,20.5f,28.6f,42.9f,62.4f,76.1f,65.0f},{7.2f,14.4f,22.8f,31.8f,47.6f,69.4f,84.5f,72.2f} },
    { {13.5f,27.0f,42.5f,59.4f,89.1f,129.6f,158.0f,135.0f},{15.0f,30.0f,47.3f,66.0f,99.0f,144.0f,175.5f,150.0f} },
    { {27.0f,54.0f,85.1f,118.8f,178.2f,259.2f,315.9f,270.0f},{30.0f,60.0f,94.5f,132.0f,198.0f,288.0f,351.0f,300.0f} }
};

static bool is_stricter(LinkThreshold lower, LinkThreshold higher, bool legacy) {
    if (!lower.valid || !higher.valid) return false;
    if (lower.rssi1 != 0 && higher.rssi1 != 0 && lower.rssi1 > higher.rssi1) return true;
    if (lower.snr1  != 0 && higher.snr1  != 0 && lower.snr1  > higher.snr1)  return true;
    if (!legacy) {
        if (lower.evm1 != 0 && higher.evm1 != 0 && lower.evm1 < higher.evm1) return true;
        if (lower.evm2 != 0 && higher.evm2 != 0 && lower.evm2 < higher.evm2) return true;
    }
    return false;
}

static bool is_identical(LinkThreshold a, LinkThreshold b, bool legacy) {
    if (!a.valid || !b.valid) return false;
    if (a.rssi1 == b.rssi1 && a.snr1 == b.snr1) {
        if (legacy) return true;
        if (a.evm1 == b.evm1 && a.evm2 == b.evm2) return true;
    }
    return false;
}

static int calculate_safe_bitrate(int mcs, int bw, int fec_k, int fec_n, int gi) {
    if (mcs < 0 || mcs > 7) return 0;
    if (gi < 0  || gi > 1)  return 0;
    int bw_idx = -1;
    switch (bw) {
        case 5:  bw_idx = 0; break; case 10: bw_idx = 1; break;
        case 20: bw_idx = 2; break; case 40: bw_idx = 3; break;
        case 80: bw_idx = 4; break; default: return 0;
    }
    float base_rate        = BASE_RATES[bw_idx][gi][mcs];
    float usable           = base_rate - (base_rate * cfg.overhead_ratio);
    float fec_overhead     = (fec_n > 0 && fec_k <= fec_n)
                             ? (float)(fec_n - fec_k) / (float)fec_n
                             : 0.0f;
    float max_app_mbps     = usable - (usable * fec_overhead);
    return (int)((max_app_mbps * cfg.safety_margin) * 1000.0f);
}

static void update_downlink_bitrate(downlink *d) {
    d->bitrate = calculate_safe_bitrate(d->mcs, d->bw, d->feck, d->fecn, d->gi);
}

/* ------------------------------------------------------------------ */
/* apply_link_settings — wfb-ng + iw + venc                           */
/* ------------------------------------------------------------------ */
static void apply_link_settings(SharedData *state, const downlink *d,
                                int raw_power_override,
                                int lost_pkts, int recovered) {
    static downlink prev = { .mcs = -1, .bitrate = -1, .feck = -1,
                             .fecn = -1, .bw = -1, .gi = -1 };
    char cmd[256];
    int mcs_safe = (d->mcs >= 0 && d->mcs <= 7) ? d->mcs : 0;
    int lvl_safe = (cfg.power_level >= 0 && cfg.power_level <= 4) ? cfg.power_level : 4;
    int pwr      = (raw_power_override != -1)
                   ? raw_power_override
                   : cfg.raw_pwr_matrix[lvl_safe][mcs_safe];

    bool is_uplink = (prev.mcs != -1) && (d->mcs > prev.mcs);

    if (is_uplink) {
        /* Going up: raise power first, then MCS, then reduce bitrate */
        snprintf(cmd, sizeof(cmd), "iw dev wlan0 set txpower fixed %d", pwr);
        system(cmd); usleep(cfg.cmd_delay_us);

        if (d->mcs != prev.mcs || d->bw != prev.bw || d->gi != prev.gi) {
            snprintf(cmd, sizeof(cmd),
                "wfb_tx_cmd 8000 set_radio -B %d -G %s -S 1 -L 1 -M %d",
                d->bw, (d->gi == 1) ? "short" : "long", d->mcs);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        if (d->feck != prev.feck || d->fecn != prev.fecn) {
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_fec -k %d -n %d", d->feck, d->fecn);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        /* bitrate via venc — done below */
    } else {
        /* Going down: lower bitrate first, then MCS, then reduce power */
        /* bitrate via venc — done below (before radio change) */
        venc_apply_for_mcs(state, d, lost_pkts, recovered);
        usleep(cfg.cmd_delay_us);

        if (d->feck != prev.feck || d->fecn != prev.fecn) {
            snprintf(cmd, sizeof(cmd), "wfb_tx_cmd 8000 set_fec -k %d -n %d", d->feck, d->fecn);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        if (d->mcs != prev.mcs || d->bw != prev.bw || d->gi != prev.gi) {
            snprintf(cmd, sizeof(cmd),
                "wfb_tx_cmd 8000 set_radio -B %d -G %s -S 1 -L 1 -M %d",
                d->bw, (d->gi == 1) ? "short" : "long", d->mcs);
            system(cmd); usleep(cfg.cmd_delay_us);
        }
        snprintf(cmd, sizeof(cmd), "iw dev wlan0 set txpower fixed %d", pwr);
        system(cmd); usleep(cfg.cmd_delay_us);

        /* Request IDR after downgrade so decoder can resync cleanly */
        if (prev.mcs != -1 && d->mcs < prev.mcs) {
            usleep(cfg.cmd_delay_us);
            venc_request_idr();
        }
        goto DONE;
    }

    /* For uplink path, apply venc after radio is configured */
    venc_apply_for_mcs(state, d, lost_pkts, recovered);

DONE:
    prev = *d;
    printf(">> [SYSTEM] MCS:%d BW:%d GI:%s FEC:%d/%d Bitrate:%dkbps Pwr:%d\n",
        d->mcs, d->bw, (d->gi == 1) ? "short" : "long",
        d->feck, d->fecn, d->bitrate, pwr);
}

static void downlink_init(SharedData *state, downlink *d) {
    d->mcs  = 7;
    d->feck = cfg.fec_k_med_protection;
    d->fecn = cfg.fec_n_constant;
    d->bw   = 20;
    d->roi  = 0;
    d->gi   = 0;
    update_downlink_bitrate(d);
    apply_link_settings(state, d, -1, 0, 0);
}

/* ------------------------------------------------------------------ */
/* Parser                                                               */
/* ------------------------------------------------------------------ */
static bool parse_telemetry_string(const char *msg, GsTelemetry *t) {
    if (!msg || !t) return false;
    memset(t, 0, sizeof(*t));
    t->evm1 = 0; t->evm2 = 0; t->rssi1 = -105;
    char *copy = strdup(msg);
    if (!copy) return false;
    int index = 0;
    char *tok = strtok(copy, ":");
    while (tok) {
        switch (index) {
            case 0: t->transmitted_time = atoi(tok); break;
            case 1: t->evm1             = atoi(tok); break;
            case 2: t->evm2             = atoi(tok); break;
            case 3: t->recovered        = atoi(tok); break;
            case 4: t->lost_packets     = atoi(tok); break;
            case 5: t->rssi1            = atoi(tok); break;
            case 6: t->snr1             = atoi(tok); break;
            case 7: t->num_antennas     = atoi(tok); break;
            case 8: t->noise_pnlty      = atoi(tok); break;
            case 9: t->fec_change       = atoi(tok); break;
            case 10:
                strncpy(t->idr_code, tok, sizeof(t->idr_code) - 1);
                t->idr_code[sizeof(t->idr_code) - 1] = '\0';
                break;
        }
        tok = strtok(NULL, ":");
        index++;
    }
    free(copy);
    return (index >= 10);
}

static GsMessageType parse_gs_packet(const uint8_t *buf, size_t len,
                                     GsTelemetry *t_out,
                                     char *special_out, size_t special_max) {
    if (len < sizeof(uint32_t)) return MSG_TYPE_INVALID;
    uint32_t msg_len;
    memcpy(&msg_len, buf, sizeof(msg_len));
    msg_len = ntohl(msg_len);
    if (msg_len > (len - sizeof(uint32_t))) return MSG_TYPE_INVALID;
    const char *payload = (const char *)(buf + sizeof(uint32_t));
    if (strncmp(payload, "special:", 8) == 0) {
        if (special_out) {
            strncpy(special_out, payload, special_max - 1);
            special_out[special_max - 1] = '\0';
        }
        return MSG_TYPE_SPECIAL;
    }
    if (parse_telemetry_string(payload, t_out)) return MSG_TYPE_TELEMETRY;
    return MSG_TYPE_INVALID;
}

/* ------------------------------------------------------------------ */
/* OSD thread                                                           */
/* ------------------------------------------------------------------ */
static void *periodic_update_osd(void *arg) {
    SharedData *shared = (SharedData *)arg;
    struct sockaddr_in udp_out_addr;
    bool udp_enabled = (shared->osd_config.udp_out_sock != -1);
    if (udp_enabled) {
        memset(&udp_out_addr, 0, sizeof(udp_out_addr));
        udp_out_addr.sin_family = AF_INET;
        udp_out_addr.sin_port   = htons(shared->osd_config.udp_out_port);
        inet_pton(AF_INET, shared->osd_config.udp_out_ip, &udp_out_addr.sin_addr);
    }

    while (true) {
        sleep(1);
        if (cfg.osd_level == 0) continue;

        pthread_mutex_lock(&shared->lock);
        int mcs        = shared->dl.mcs;
        int feck       = shared->dl.feck;
        int fecn       = shared->dl.fecn;
        int evm        = shared->telemetry.evm1;
        int rssi       = shared->telemetry.rssi1;
        int snr        = shared->telemetry.snr1;
        int lost       = shared->telemetry.lost_packets;
        int rec        = shared->telemetry.recovered;
        int cooldown   = shared->cooldown_ticks;
        int noise_floor= rssi - snr;
        bool legacy    = shared->legacy_mode;
        bool failsafe  = shared->failsafe_mode;
        bool corrupted = shared->log_corrupted;
        bool incons    = shared->log_inconsistent;
        bool is_prob   = shared->is_probing;
        bool is_pred   = (shared->predict.state != PREDICT_IDLE);
        char p_msg[64];
        strncpy(p_msg, shared->predict.status_msg, sizeof(p_msg) - 1);
        int cur_gop_x100 = (int)(shared->last_venc_gop * 100.0f);
        int cur_roi_qp   = shared->last_roi_qp;
        LinkThreshold local_th[8];
        memcpy(local_th, shared->display_th, sizeof(local_th));
        pthread_mutex_unlock(&shared->lock);

        char rdy_str[16];
        if      (is_pred) strcpy(rdy_str, "CAL");
        else if (is_prob) strcpy(rdy_str, "PRB");
        else if (cooldown > 0) strcpy(rdy_str, "WAIT");
        else    strcpy(rdy_str, "RDY");

        char mode_str[128];
        char err_str[64] = "";
        if      (is_pred)   strncpy(err_str, p_msg, sizeof(err_str)-1);
        else if (corrupted) strcpy(err_str, "[CORRUPTED LOG]");
        else if (incons)    strcpy(err_str, "[LOG INCONSISTENCY]");

        snprintf(mode_str, sizeof(mode_str), "Modes: %s / %s %s",
            legacy ? "LEGACY" : "STANDARD",
            failsafe ? "FAILSAFE" : "NORMAL", err_str);

        char log_str[1024] = "LOG [M:V:E1:E2:R:S:U:P:PWR]\n";
        for (int i = 7; i >= 0; i--) {
            char ln[64];
            if (local_th[i].valid) {
                snprintf(ln, sizeof(ln), "%d:1:%d:%d:%d:%d:%d:%d:%d\n",
                    i, local_th[i].evm1, local_th[i].evm2,
                    local_th[i].rssi1, local_th[i].snr1,
                    local_th[i].can_uplink, local_th[i].probe_success_count,
                    local_th[i].rawpower);
            } else {
                snprintf(ln, sizeof(ln), "%d:0:0:0:0:0:0:0:0\n", i);
            }
            strncat(log_str, ln, sizeof(log_str) - strlen(log_str) - 1);
        }

        char full_osd[2048];
        snprintf(full_osd, sizeof(full_osd),
            "&L%d0&F%d MCS:%d FEC:%d/%d EVM:%d RSSI:%d SNR:%d NF:%d "
            "LST:%d REC:%d GOP:%.2fs ROI:%d [%s]\n%s\n%s",
            cfg.osd_colour, cfg.osd_font_size,
            mcs, feck, fecn, evm, rssi, snr, noise_floor,
            lost, rec,
            (double)(cur_gop_x100) / 100.0, cur_roi_qp,
            rdy_str, mode_str, log_str);

        if (udp_enabled) {
            sendto(shared->osd_config.udp_out_sock, full_osd, strlen(full_osd), 0,
                (struct sockaddr *)&udp_out_addr, sizeof(udp_out_addr));
        } else {
            FILE *f = fopen("/tmp/MSPOSD.msg", "w");
            if (f) { fwrite(full_osd, 1, strlen(full_osd), f); fclose(f); }
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* FIFO command listener                                                */
/* ------------------------------------------------------------------ */
static void *fifo_listener_thread(void *arg) {
    SharedData *shared = (SharedData *)arg;
    mkfifo(FIFO_PATH, 0666);
    char buf[128];
    while (1) {
        int fd = open(FIFO_PATH, O_RDONLY);
        if (fd < 0) { sleep(1); continue; }
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            if (buf[n-1] == '\n') buf[n-1] = '\0';
            pthread_mutex_lock(&shared->lock);

            if (strncmp(buf, "delete_log", 10) == 0) {
                unlink(LOG_FILE_PATH);
                for (int k = 0; k <= 7; k++) shared->display_th[k].valid = false;
                shared->failsafe_mode   = true;
                shared->log_corrupted   = true;
                shared->corruption_time = time(NULL);
                printf(">> [CMD] Log deleted. Failsafe activated.\n");
            } else if (strncmp(buf, "predict", 7) == 0) {
                if (shared->predict.state == PREDICT_IDLE) {
                    shared->predict.state = PREDICT_START_MAX;
                    printf(">> [CMD] Calibration routine started.\n");
                }
            } else if (strncmp(buf, "set_powerlevel", 14) == 0) {
                int plvl;
                if (sscanf(buf + 14, "%d", &plvl) == 1 && plvl >= 0 && plvl <= 4) {
                    cfg.power_level = plvl;
                    printf(">> [CMD] Power level → %d\n", plvl);
                }
            } else if (strncmp(buf, "request_idr", 11) == 0) {
                /* Direct IDR request via FIFO */
                venc_request_idr();
            } else if (strncmp(buf, "set_gop_good", 12) == 0) {
                float g;
                if (sscanf(buf + 12, "%f", &g) == 1 && g > 0.0f) {
                    cfg.gop_good_link = g;
                    printf(">> [CMD] GOP good link → %.2fs\n", (double)g);
                }
            } else if (strncmp(buf, "set_gop_bad", 11) == 0) {
                float g;
                if (sscanf(buf + 11, "%f", &g) == 1 && g > 0.0f) {
                    cfg.gop_bad_link = g;
                    printf(">> [CMD] GOP bad link → %.2fs\n", (double)g);
                }
            }

            pthread_mutex_unlock(&shared->lock);
        }
        close(fd);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* UDP telemetry listener                                               */
/* ------------------------------------------------------------------ */
static void *udp_listener_thread(void *arg) {
    SharedData *shared = (SharedData *)arg;
    struct sockaddr_in srv, cli;
    socklen_t cli_len = sizeof(cli);
    uint8_t buffer[BUFFER_SIZE];

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) { perror("socket"); pthread_exit(NULL); }

    memset(&srv, 0, sizeof(srv));
    srv.sin_family      = AF_INET;
    srv.sin_addr.s_addr = INADDR_ANY;
    srv.sin_port        = htons(LISTEN_PORT);

    if (bind(sockfd, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
        perror("bind"); close(sockfd); pthread_exit(NULL);
    }

    char special_cmd[256];
    GsTelemetry temp_t;

    while (1) {
        int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                         (struct sockaddr *)&cli, &cli_len);
        if (n < 0) break;
        buffer[n] = '\0';
        GsMessageType mt = parse_gs_packet(buffer, n, &temp_t,
                                           special_cmd, sizeof(special_cmd));
        switch (mt) {
            case MSG_TYPE_TELEMETRY:
                pthread_mutex_lock(&shared->lock);
                shared->telemetry   = temp_t;
                shared->has_new_data = true;
                if (temp_t.evm1 > 45 || temp_t.evm2 > 45)
                    shared->legacy_mode = true;
                pthread_mutex_unlock(&shared->lock);
                break;
            case MSG_TYPE_SPECIAL:
                if (strncmp(special_cmd, "special:request_keyframe", 24) == 0)
                    venc_request_idr();
                break;
            default: break;
        }
    }
    close(sockfd);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Log management                                                       */
/* ------------------------------------------------------------------ */
static bool validate_logs(LinkThreshold *th, bool legacy_mode) {
    for (int i = 0; i < 7; i++) {
        if (!th[i].valid) continue;
        for (int j = i + 1; j <= 7; j++) {
            if (!th[j].valid) continue;
            if (is_identical(th[i], th[j], legacy_mode)) return false;
            if (is_stricter(th[i], th[j], legacy_mode))  return false;
        }
    }
    return true;
}

static void load_logs(LinkThreshold *th, bool *legacy_mode) {
    for (int i = 0; i <= 7; i++) {
        th[i].valid = false; th[i].can_uplink = 0;
        th[i].probe_success_count = 0; th[i].rawpower = 0;
    }
    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (!f) return;
    char header[32];
    int legacy_val = 0;
    if (fscanf(f, "%31s %d", header, &legacy_val) == 2 &&
        strcmp(header, "LEGACY") == 0)
        *legacy_mode = (legacy_val == 1);
    else
        rewind(f);
    int mcs, valid, evm1, evm2, rssi1, snr1, cu, ps, rp;
    while (fscanf(f, "%d %d %d %d %d %d %d %d %d",
                  &mcs, &valid, &evm1, &evm2, &rssi1, &snr1, &cu, &ps, &rp) == 9) {
        if (mcs >= 0 && mcs <= 7) {
            th[mcs].valid = valid; th[mcs].evm1 = evm1; th[mcs].evm2 = evm2;
            th[mcs].rssi1 = rssi1; th[mcs].snr1 = snr1;
            th[mcs].can_uplink = cu; th[mcs].probe_success_count = ps;
            th[mcs].rawpower = rp;
        }
    }
    fclose(f);
}

static void save_logs(LinkThreshold *th, bool legacy_mode) {
    char tmp[256];
    snprintf(tmp, sizeof(tmp), "%s.tmp", LOG_FILE_PATH);
    FILE *f = fopen(tmp, "w");
    if (!f) return;
    fprintf(f, "LEGACY %d\n", legacy_mode ? 1 : 0);
    for (int i = 0; i <= 7; i++) {
        fprintf(f, "%d %d %d %d %d %d %d %d %d\n",
            i, th[i].valid, th[i].evm1, th[i].evm2,
            th[i].rssi1, th[i].snr1, th[i].can_uplink,
            th[i].probe_success_count, th[i].rawpower);
    }
    fflush(f); fsync(fileno(f)); fclose(f);
    rename(tmp, LOG_FILE_PATH);
}

/* ------------------------------------------------------------------ */
/* Prediction routine                                                   */
/* ------------------------------------------------------------------ */
static bool handle_prediction(SharedData *state, LinkThreshold *th) {
    PredictEngine *pe = &state->predict;
    int required_ticks = cfg.predict_time_s * 40; /* 40 Hz loop */

    switch (pe->state) {
        case PREDICT_START_MAX:
            strcpy(pe->status_msg, "[CAL: MAX PWR]");
            state->dl.mcs  = 0;
            state->dl.feck = cfg.fec_k_med_protection;
            update_downlink_bitrate(&state->dl);
            apply_link_settings(state, &state->dl, cfg.raw_pwr_matrix[4][0], 0, 0);
            pe->ticks_counter = 0; pe->sum_nf = 0; pe->sample_count = 0;
            pe->state = PREDICT_WAIT_MAX;
            return true;

        case PREDICT_WAIT_MAX:
            if (state->has_new_data &&
                state->telemetry.rssi1 != 0 && state->telemetry.snr1 != 0) {
                pe->sum_nf += (state->telemetry.rssi1 - state->telemetry.snr1);
                pe->sample_count++;
            }
            if (++pe->ticks_counter >= required_ticks) {
                pe->max_pwr_nf = (pe->sample_count > 0)
                                 ? (int)(pe->sum_nf / pe->sample_count) : -100;
                pe->state = PREDICT_START_MIN;
            }
            return true;

        case PREDICT_START_MIN:
            strcpy(pe->status_msg, "[CAL: MIN PWR]");
            apply_link_settings(state, &state->dl, cfg.raw_pwr_matrix[0][0], 0, 0);
            pe->ticks_counter = 0; pe->sum_nf = 0; pe->sample_count = 0;
            pe->state = PREDICT_WAIT_MIN;
            return true;

        case PREDICT_WAIT_MIN:
            if (state->has_new_data &&
                state->telemetry.rssi1 != 0 && state->telemetry.snr1 != 0) {
                pe->sum_nf += (state->telemetry.rssi1 - state->telemetry.snr1);
                pe->sample_count++;
            }
            if (++pe->ticks_counter >= required_ticks) {
                pe->baseline_nf = (pe->sample_count > 0)
                                  ? (int)(pe->sum_nf / pe->sample_count) : -100;
                if (pe->baseline_nf > -75) {
                    printf(">> [PREDICT] Too noisy (%d). Aborting.\n", pe->baseline_nf);
                    strcpy(pe->status_msg, "[CAL: TOO NOISY]");
                    pe->state = PREDICT_IDLE;
                    return true;
                }
                if (pe->max_pwr_nf - pe->baseline_nf > cfg.predict_delta) {
                    pe->test_pwr          = cfg.raw_pwr_matrix[0][0];
                    pe->max_safe_close_pwr = pe->test_pwr;
                    pe->state             = PREDICT_STEP_PWR;
                    strcpy(pe->status_msg, "[CAL: STEPPING]");
                    apply_link_settings(state, &state->dl, pe->test_pwr, 0, 0);
                    pe->ticks_counter = 0;
                } else {
                    pe->max_safe_close_pwr = cfg.raw_pwr_matrix[4][0];
                    pe->state = PREDICT_DONE;
                }
            }
            return true;

        case PREDICT_STEP_PWR:
            if (++pe->ticks_counter >= 10 && state->has_new_data) {
                int cur_nf = state->telemetry.rssi1 - state->telemetry.snr1;
                if (cur_nf - pe->baseline_nf < cfg.predict_delta) {
                    pe->max_safe_close_pwr = pe->test_pwr;
                    pe->test_pwr += 100;
                    apply_link_settings(state, &state->dl, pe->test_pwr, 0, 0);
                    pe->ticks_counter = 0;
                } else {
                    pe->state = PREDICT_DONE;
                }
            }
            return true;

        case PREDICT_DONE: {
            strcpy(pe->status_msg, "");
            printf(">> [PREDICT] Complete. Baseline NF: %d  Max safe pwr: %d\n",
                pe->baseline_nf, pe->max_safe_close_pwr);
            int nf_calc = pe->baseline_nf + 2;
            for (int i = 0; i <= 7; i++) {
                th[i].valid = true; th[i].can_uplink = true;
                th[i].probe_success_count = 0;
                th[i].evm1 = 0; th[i].evm2 = 0;
                th[i].snr1  = cfg.snr_mcs[i];
                th[i].rssi1 = nf_calc + cfg.snr_mcs[i];
                int lvl = (cfg.power_level >= 0 && cfg.power_level <= 4) ? cfg.power_level : 4;
                int bp  = cfg.raw_pwr_matrix[lvl][i];
                if (i <= cfg.predict_low_mcs_max) {
                    th[i].rawpower = bp;
                } else if (i >= cfg.predict_high_mcs_min) {
                    th[i].rawpower = (bp > pe->max_safe_close_pwr)
                                     ? pe->max_safe_close_pwr : bp;
                } else {
                    th[i].rawpower = bp;
                }
            }
            save_logs(th, state->legacy_mode);
            state->log_corrupted   = false;
            state->log_inconsistent = false;
            state->failsafe_mode   = false;
            apply_link_settings(state, &state->dl, -1, 0, 0);
            pe->state = PREDICT_IDLE;
            return true;
        }

        default: return false;
    }
}

/* ------------------------------------------------------------------ */
/* Main link logic                                                      */
/* ------------------------------------------------------------------ */
static bool g0ylink(const GsTelemetry *t, downlink *d, LinkThreshold *th,
                    SharedData *state) {
    if (state->predict.state != PREDICT_IDLE) {
        handle_prediction(state, th);
        return false;
    }

    bool log_saved        = false;
    bool settings_changed = false;

    if (!state->log_corrupted ||
        (time(NULL) - state->corruption_time > 5)) {
        memcpy(state->display_th, th, sizeof(LinkThreshold) * 8);
        if (state->log_corrupted) state->log_corrupted = false;
    }
    if (!state->log_inconsistent ||
        (time(NULL) - state->corruption_time > 5)) {
        if (state->log_inconsistent) state->log_inconsistent = false;
    }

    /* Integrity check */
    for (int i = 0; i < 7; i++) {
        for (int j = i + 1; j <= 7; j++) {
            if (is_identical(th[i], th[j], state->legacy_mode)) {
                unlink(LOG_FILE_PATH);
                for (int k = 0; k <= 7; k++) th[k].valid = false;
                state->failsafe_mode   = true;
                state->log_corrupted   = true;
                state->corruption_time = time(NULL);
                memcpy(state->display_th, th, sizeof(LinkThreshold) * 8);
                return false;
            }
        }
    }

    if (state->cooldown_ticks > 0) state->cooldown_ticks--;

    /* ---- FAILSAFE MODE ---- */
    if (state->failsafe_mode) {
        if (d->mcs != 0 || d->feck != cfg.fec_k_failsafe) {
            d->mcs  = 0;
            d->feck = cfg.fec_k_failsafe;
            update_downlink_bitrate(d);
            apply_link_settings(state, d, -1, t->lost_packets, t->recovered);
        }
        /* Check whether we can escape failsafe */
        int target_mcs = -1;
        for (int i = 1; i <= 7; i++) {
            if (th[i].valid && th[i].can_uplink) { target_mcs = i; break; }
        }
        bool can_escape = false;
        if (target_mcs != -1) {
            bool ev1 = state->legacy_mode || (t->evm1 == 0) || (t->evm1 < th[target_mcs].evm1);
            bool ev2 = state->legacy_mode || (t->evm2 == 0) || (t->evm2 < th[target_mcs].evm2);
            bool rs  = (t->rssi1 == 0) || (t->rssi1 > th[target_mcs].rssi1);
            bool sn  = (t->snr1  == 0) || (t->snr1  > th[target_mcs].snr1);
            bool lp  = (t->lost_packets <= cfg.uplink_lost_pkts_thresh);
            if (ev1 && ev2 && rs && sn && lp) can_escape = true;
        } else {
            if ((t->snr1 == 0 || t->snr1 > 25) &&
                t->lost_packets <= cfg.uplink_lost_pkts_thresh)
                can_escape = true;
            target_mcs = 1;
        }
        if (can_escape) {
            state->uplink_stable_count++;
            if (state->uplink_stable_count >= cfg.uplink_stability_ticks) {
                state->failsafe_mode = false;
                d->mcs               = target_mcs;
                state->cooldown_ticks = cfg.change_cooldown_ticks;
                state->uplink_stable_count = 0;
                settings_changed = true;
            }
        } else {
            state->uplink_stable_count = 0;
        }
        if (settings_changed) {
            update_downlink_bitrate(d);
            int pwr = (th[d->mcs].valid && th[d->mcs].rawpower > 0)
                      ? th[d->mcs].rawpower : -1;
            apply_link_settings(state, d, pwr, t->lost_packets, t->recovered);
        }
        return false;
    }

    /* ---- PROBING MODE ---- */
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
                    /* Probe successful */
                    state->is_probing = false;
                    state->probe_failed_count[state->probe_target_mcs] = 0;
                    th[state->probe_target_mcs].probe_success_count++;
                    LinkThreshold *tgt = &th[state->probe_target_mcs];
                    bool worse = false;
                    if (!tgt->valid) {
                        worse = true;
                    } else {
                        if (t->rssi1 != 0 && tgt->rssi1 != 0 && t->rssi1 < tgt->rssi1) worse = true;
                        if (t->snr1  != 0 && tgt->snr1  != 0 && t->snr1  < tgt->snr1)  worse = true;
                        if (!state->legacy_mode && t->evm1 != 0 && tgt->evm1 != 0 && t->evm1 > tgt->evm1) worse = true;
                    }
                    if (worse) {
                        tgt->valid = true;
                        tgt->evm1  = t->evm1  + cfg.offset_evm1;
                        tgt->evm2  = t->evm2  + cfg.offset_evm2;
                        tgt->rssi1 = t->rssi1 + cfg.offset_rssi1;
                        tgt->snr1  = t->snr1  + cfg.offset_snr1;
                        tgt->can_uplink = 1;
                        if (tgt->rawpower == 0) {
                            int lvl = (cfg.power_level >= 0 && cfg.power_level <= 4) ? cfg.power_level : 4;
                            tgt->rawpower = cfg.raw_pwr_matrix[lvl][state->probe_target_mcs];
                        }
                        /* Clamp thresholds so higher MCS is strictly harder to reach */
                        for (int j = state->probe_target_mcs + 1; j <= 7; j++) {
                            if (!th[j].valid) continue;
                            if (tgt->rssi1 >= th[j].rssi1) tgt->rssi1 = th[j].rssi1 - 1;
                            if (tgt->snr1  >= th[j].snr1)  tgt->snr1  = th[j].snr1  - 1;
                            if (!state->legacy_mode) {
                                if (tgt->evm1 <= th[j].evm1) tgt->evm1 = th[j].evm1 + 1;
                                if (tgt->evm2 <= th[j].evm2) tgt->evm2 = th[j].evm2 + 1;
                            }
                        }
                        /* Invalidate lower MCS if it becomes too conservative */
                        if (state->probe_original_mcs >= 0 &&
                            th[state->probe_original_mcs].valid) {
                            LinkThreshold *lwr = &th[state->probe_original_mcs];
                            bool too_cons = false;
                            if (tgt->rssi1 < lwr->rssi1) too_cons = true;
                            if (tgt->snr1  < lwr->snr1)  too_cons = true;
                            if (!state->legacy_mode && tgt->evm1 > lwr->evm1) too_cons = true;
                            if (too_cons) { lwr->valid = false; lwr->can_uplink = 0; }
                        }
                        log_saved = true;
                    }
                }
            }
        }
        goto APPLY_CHANGES_BLOCK;
    }

    /* ---- NORMAL MODE — FEC adaptation ---- */
    if (!state->is_probing && !state->failsafe_mode) {
        int target_feck = d->feck;
        if (t->recovered >= cfg.fec_recovered_thresh_high)
            target_feck = cfg.fec_k_high_protection;
        else if (t->recovered <= cfg.fec_recovered_thresh_low)
            target_feck = cfg.fec_k_low_protection;
        else
            target_feck = cfg.fec_k_med_protection;
        if (target_feck != d->feck) { d->feck = target_feck; settings_changed = true; }
    }

    if (state->cooldown_ticks > 0) goto APPLY_CHANGES_BLOCK;

    int old_mcs = d->mcs;

    /* ---- Uplink probing trigger ---- */
    if (d->mcs < 7 &&
        t->lost_packets == 0 && t->recovered <= 1 &&
        th[d->mcs + 1].probe_success_count < cfg.max_probe_success_count) {
        int tgt_mcs  = d->mcs + 1;
        int penalty  = state->probe_failed_count[tgt_mcs];
        int threshold = cfg.snr_mcs[tgt_mcs] - 2 + penalty;
        if (t->snr1 != 0 && t->snr1 >= threshold) {
            state->probe_wait_ticks++;
            if (state->probe_wait_ticks >= cfg.probing_stability_ticks) {
                state->is_probing          = true;
                state->probe_target_mcs    = tgt_mcs;
                state->probe_original_mcs  = d->mcs;
                state->probe_step_ticks    = 0;
                d->mcs  = tgt_mcs;
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

    /* ---- Downlink threshold ---- */
    int cur_thresh = cfg.downlink_lost_pkts_thresh;
    if (old_mcs >= 6) {
        cur_thresh = th[old_mcs].valid
                     ? cfg.downlink_lost_pkts67_c_thresh
                     : cfg.downlink_lost_pkts67_thresh;
    }

    /* Detect inconsistency */
    if (th[old_mcs].valid) {
        for (int j = old_mcs + 1; j <= 7; j++) {
            if (is_stricter(th[old_mcs], th[j], state->legacy_mode)) {
                th[old_mcs].valid   = false;
                th[old_mcs].can_uplink = 0;
                state->failsafe_mode    = true;
                state->log_inconsistent = true;
                state->corruption_time  = time(NULL);
                memcpy(state->display_th, th, sizeof(LinkThreshold) * 8);
                return true;
            }
        }
    }

    if (th[old_mcs].valid) {
        int bad_evm1  = th[old_mcs].evm1  - cfg.offset_evm1;
        int bad_evm2  = th[old_mcs].evm2  - cfg.offset_evm2;
        int bad_rssi1 = th[old_mcs].rssi1 - cfg.offset_rssi1;
        int bad_snr1  = th[old_mcs].snr1  - cfg.offset_snr1;

        bool evm1_bad  = !state->legacy_mode && (t->evm1  != 0) && (t->evm1  >= bad_evm1);
        bool evm2_bad  = !state->legacy_mode && (t->evm2  != 0) && (t->evm2  >= bad_evm2);
        bool rssi_bad  = (t->rssi1 != 0) && (t->rssi1 <= bad_rssi1);
        bool snr_bad   = (t->snr1  != 0) && (t->snr1  <= bad_snr1);
        bool pkts_bad  = (t->lost_packets > cur_thresh);

        if (evm1_bad || evm2_bad || rssi_bad || snr_bad || pkts_bad) {
            /* Downgrade */
            int new_mcs = old_mcs - 1;
            if (new_mcs < 0) new_mcs = 0;
            d->mcs  = new_mcs;
            d->feck = cfg.fec_k_high_protection;
            state->cooldown_ticks = cfg.change_cooldown_ticks;
            settings_changed = true;
        }
    } else if (t->lost_packets > cur_thresh) {
        /* No threshold data — step down conservatively */
        if (d->mcs > 0) {
            d->mcs--;
            d->feck = cfg.fec_k_high_protection;
            state->cooldown_ticks = cfg.change_cooldown_ticks;
            settings_changed = true;
        }
    }

APPLY_CHANGES_BLOCK:
    if (log_saved) {
        save_logs(th, state->legacy_mode);
        memcpy(state->display_th, th, sizeof(LinkThreshold) * 8);
    }
    if (settings_changed) {
        update_downlink_bitrate(d);
        int pwr = (th[d->mcs].valid && th[d->mcs].rawpower > 0)
                  ? th[d->mcs].rawpower : -1;
        apply_link_settings(state, d, pwr, t->lost_packets, t->recovered);
        return true;
    }

    /* Even when nothing changed, keep venc in sync (rate-limited internally) */
    venc_apply_for_mcs(state, d, t->lost_packets, t->recovered);

    return false;
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */
int main(int argc, char *argv[]) {
    (void)argc; (void)argv;

    load_config();

    SharedData state;
    memset(&state, 0, sizeof(state));
    pthread_mutex_init(&state.lock, NULL);

    /* Init venc tracking to sentinel values */
    state.last_venc_bitrate = -1;
    state.last_venc_gop     = -1.0f;
    state.last_roi_qp       = INT32_MIN; /* force first update */

    /* OSD: try UDP first, fall back to file */
    state.osd_config.udp_out_sock = -1;
    int osd_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (osd_sock >= 0) {
        state.osd_config.udp_out_sock = osd_sock;
        strncpy(state.osd_config.udp_out_ip, "127.0.0.1",
                sizeof(state.osd_config.udp_out_ip) - 1);
        state.osd_config.udp_out_port = 14551; /* MSPOSD default */
    }

    /* Load persisted thresholds */
    LinkThreshold th[8];
    load_logs(th, &state.legacy_mode);
    if (!validate_logs(th, state.legacy_mode)) {
        printf(">> [STARTUP] Log validation failed. Entering failsafe.\n");
        for (int i = 0; i <= 7; i++) th[i].valid = false;
        state.failsafe_mode   = true;
        state.log_corrupted   = true;
        state.corruption_time = time(NULL);
    }
    memcpy(state.display_th, th, sizeof(th));

    /* Init downlink */
    downlink_init(&state, &state.dl);

    /* Spin threads */
    pthread_t t_osd, t_fifo, t_udp;
    pthread_create(&t_osd,  NULL, periodic_update_osd,   &state);
    pthread_create(&t_fifo, NULL, fifo_listener_thread,  &state);
    pthread_create(&t_udp,  NULL, udp_listener_thread,   &state);

    printf(">> [FLUKE] Running. Listening on UDP %d. venc at http://%s:%d\n",
        LISTEN_PORT, cfg.venc_ip, cfg.venc_port);

    /* Main 40 Hz loop */
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 25000000L }; /* 25 ms */

    while (1) {
        nanosleep(&ts, NULL);

        pthread_mutex_lock(&state.lock);

        if (!state.has_new_data) {
            state.no_telemetry_ticks++;
            if (state.no_telemetry_ticks >= cfg.no_telemetry_ticks_thresh) {
                if (!state.failsafe_mode) {
                    printf(">> [WARN] No telemetry — entering failsafe.\n");
                    state.failsafe_mode = true;
                    state.dl.mcs  = 0;
                    state.dl.feck = cfg.fec_k_failsafe;
                    update_downlink_bitrate(&state.dl);
                    apply_link_settings(&state, &state.dl, -1, 0, 0);
                }
            }
        } else {
            state.no_telemetry_ticks = 0;
            state.has_new_data       = false;

            GsTelemetry t_snap = state.telemetry;
            g0ylink(&t_snap, &state.dl, th, &state);
        }

        pthread_mutex_unlock(&state.lock);
    }

    /* Unreachable — but tidy */
    pthread_mutex_destroy(&state.lock);
    return 0;
}
