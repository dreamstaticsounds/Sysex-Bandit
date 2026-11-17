/* Engine-level loopback for the Sysex Bandit port (x86, live /dev/snd/seq):
 * instantiate the plugin, wire raw ALSA clients to its ports, and prove the path
 * that matters on a device --
 *   - the 2 s sweep discovers the probe's out-port (nothing subscribes the other
 *     way), so received SysEx reaches the RX readout and bumps RX TOTAL;
 *   - pressing SEND (BYTES with hand-edited bytes, FILE, then a two-message file)
 *     delivers the exact bytes to our subscriber, in order, with TX/STATUS showing them;
 *   - OUT PORT picks ONE receiver: A only, B only, ALL, and a port that is not
 *     there refuses instead of quietly reaching everybody;
 *   - IN PORT picks ONE source: a second sender is heard under ALL and unheard
 *     once the pick is on the first one;
 *   - STATUS names the client a message came from;
 *   - the echo filter drops our own bytes coming back, but only within a few
 *     seconds of sending them (a read-back minutes later is a real message);
 *   - a pick follows its PORT across list shifts: deleting a port before it
 *     leaves the pick on the same device, a vanished pick reads (gone) and
 *     refuses (in and out), and a device that comes back on a new port id is
 *     re-found by name;
 *   - RX TOTAL's change reaches the host through display_rev (audioMasterAutomate);
 *   - every message since CLEAR is kept: SAVE RX writes the whole capture (a bank
 *     dump that arrives as many ~120-byte blocks, the JD-990 case 1.0.0 lost),
 *     and RX MSG steps through it n/m, newest first;
 *   - SAVE RX writes messages/rx_001.syx and the slot list picks it up;
 *   - a bank dump (64 KB, sent as 64 fragments because ALSA cuts SysEx at 4096
 *     bytes per event) reassembles into one message and saves back byte-exact;
 *   - SEND is paced by the SPEED knobs: tx_buf-byte blocks with tx_gap ms between
 *     them land byte-exact and measurably slower than full throttle;
 *   - a file name too long for the MESSAGE box scrolls through it (display_rev
 *     moves, so the wrapper repaints);
 *   - LISTEN OFF unsubscribes (inject goes nowhere) and ON sweeps again;
 *   - MESSAGE NEXT / PREV step the slot through the wrapper's step_of;
 *   - the state chunk restores on a second instance, file= by name.
 * Run with test/engine_loop.sh. Exit 1 on failure. */
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <alsa/asoundlib.h>
#include "params.h"

typedef struct AEffect AEffect;
typedef intptr_t (*cb)(AEffect*,int32_t,int32_t,intptr_t,void*,float);
struct AEffect { int32_t magic; intptr_t (*d)(AEffect*,int32_t,int32_t,intptr_t,void*,float);
 void*p; void (*setP)(AEffect*,int32_t,float); float (*getP)(AEffect*,int32_t);
 int32_t np,npar,ni,no,flags; intptr_t r1,r2; int32_t a,b,c; float io; void*obj,*user; int32_t uid,ver;
 void (*pr)(AEffect*,float**,float**,int32_t); void*pdr; char f[56]; };
extern AEffect* VSTPluginMain(cb);

static int automated[NPARAMS], fails;
/* Evaluate the condition ONCE: side-effecting checks (snd_seq_open,
 * snd_seq_subscribe_port) must not run twice -- the second run both leaks the
 * first handle and can change the result (EEXIST) after "ok" was printed. */
#define CHECK(c, ...) do { int _r = !!(c); printf("%s ", _r ? "ok  " : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!_r) fails++; } while (0)

static intptr_t host(AEffect *e, int32_t op, int32_t i, intptr_t v, void *p, float o) {
    (void)e; (void)v; (void)p; (void)o;
    if (op == 0 && i >= 0 && i < NPARAMS) automated[i]++;
    return 0;
}

static int find_param(const char *key) {
    for (int i = 0; i < NPARAMS; i++) if (!strcmp(PARAMS[i].key, key)) return i;
    return -1;
}

static void run(AEffect *a, int blocks) {          /* instrument: NULL in */
    float L[128], R[128], *o[2] = {L, R};
    for (int k = 0; k < blocks; k++) a->pr(a, 0, o, 128);
}

static void disp(AEffect *a, int idx, char *buf, size_t n) {
    buf[0] = 0;
    a->d(a, 7, idx, 0, buf, 0);
    buf[n - 1] = 0;
}

/* ------------------------------------------------------------ probe side */

typedef struct { uint8_t b[4096]; int len; int port; } msg_t;

static snd_seq_t *pseq;      /* "Probe A": the source we inject with, the receiver we read */
static snd_seq_t *pseq_b;    /* "Probe B": a second receiver, to be left out on purpose */
static snd_seq_t *pseq_c;    /* "Probe C": a second source, to be unheard on purpose */
static int p_out = -1, p_out_c = -1;

static int sysex_send(snd_seq_t *sq, int port, const uint8_t *p, int len) {
    snd_seq_event_t ev;
    snd_seq_ev_clear(&ev);
    snd_seq_ev_set_source(&ev, port);
    snd_seq_ev_set_subs(&ev);                    /* the plugin's in-port, once swept */
    snd_seq_ev_set_direct(&ev);
    ev.type = SND_SEQ_EVENT_SYSEX;
    snd_seq_ev_set_variable(&ev, len, (void *)(uintptr_t)p);   /* LENGTH_VARIABLE or -EINVAL */
    return snd_seq_event_output_direct(sq, &ev);
}

static int probe_send(const uint8_t *p, int len) { return sysex_send(pseq, p_out, p, len); }
static int probe_c_send(const uint8_t *p, int len) { return sysex_send(pseq_c, p_out_c, p, len); }

static int drain_on(snd_seq_t *sq, msg_t *out, int max) {
    snd_seq_event_t *ev;
    int n = 0;
    while (n < max && snd_seq_event_input(sq, &ev) >= 0) {
        if (ev->type != SND_SEQ_EVENT_SYSEX) continue;
        int len = (int)ev->data.ext.len;
        if (len < 0) continue;
        if (len > (int)sizeof out[0].b) len = (int)sizeof out[0].b;
        memcpy(out[n].b, ev->data.ext.ptr, (size_t)len);
        out[n].len = len;
        out[n].port = (int)ev->dest.port;        /* which of that client's ports it landed on */
        n++;
    }
    return n;
}

static int probe_drain(msg_t *out, int max) { return drain_on(pseq, out, max); }
static int probe_b_drain(msg_t *out, int max) { return drain_on(pseq_b, out, max); }

/* How many of the drained events were for this port (ev->dest names the real
 * receiving port -- probed: ALSA rewrites it even for a set_subs broadcast). */
static int count_to(const msg_t *m, int n, int port) {
    int c = 0;
    for (int i = 0; i < n; i++) if (m[i].port == port) c++;
    return c;
}

static int is(const msg_t *m, const uint8_t *p, int len) {
    return m->len == len && !memcmp(m->b, p, (size_t)len);
}

/* Wait for the plugin's sweep to subscribe its in-port to ours: send a marker
 * every 200 ms until the plugin reports it. Returns 1 when it arrived. */
static int wait_swept(AEffect *a, int rx_total_idx) {
    static const uint8_t mark[] = {0xF0, 0x1D, 0x2E, 0x3F, 0xF7};
    float before = a->getP(a, rx_total_idx);
    for (int t = 0; t < 35; t++) {               /* up to ~7 s: two sweep periods */
        probe_send(mark, sizeof mark);
        for (int k = 0; k < 5; k++) usleep(40000);   /* 200 ms for the reader's poll */
        if (a->getP(a, rx_total_idx) > before) return 1;
    }
    return 0;
}

/* Walk a port picker's range until its display names `needle`, then put it back
 * where it was. Returns 0 when that port is not on the list. */
static int find_tg(AEffect *a, int idx, int max, const char *needle) {
    char buf[256];
    float was = a->getP(a, idx);
    int found = 0;
    for (int k = 1; k <= max && !found; k++) {
        a->setP(a, idx, (float)k / (float)max);
        disp(a, idx, buf, sizeof buf);
        if (strstr(buf, needle)) found = k;
    }
    a->setP(a, idx, was);
    return found;
}

/* The highest index that resolves to a port that exists (0 = none). */
static int last_tg(AEffect *a, int idx, int max) {
    char buf[256];
    int last = 0;
    for (int k = 1; k <= max; k++) {
        a->setP(a, idx, (float)k / (float)max);
        disp(a, idx, buf, sizeof buf);
        if (!strstr(buf, "(gone)")) last = k;
    }
    a->setP(a, idx, 0.0f);
    return last;
}

/* find_tg, but patient: the picker lists are rebuilt by the plugin's 2 s sweep,
 * so a port that has only just been plugged in may take a pass to show up. */
static int find_tg_wait(AEffect *a, int idx, int max, const char *needle) {
    for (int t = 0; t < 35; t++) {
        int k = find_tg(a, idx, max, needle);
        if (k > 0) return k;
        usleep(200000);
    }
    return 0;
}

/* Did RX TOTAL move? The reader polls every 200 ms, so up to a second. */
static int rx_seen(AEffect *a, int idx, float before) {
    for (int k = 0; k < 25; k++) {
        usleep(40000);
        if (a->getP(a, idx) > before) return 1;
    }
    return 0;
}

/* SEND only queues: STATUS stays "sending..." until the tx thread is out. Returns
 * 1 once STATUS says anything else (a refusal is already written, so it returns
 * at once), 0 if the send never reported back. */
static int wait_sent(AEffect *a, int i_st) {
    char st[256];
    for (int k = 0; k < 300; k++) {
        disp(a, i_st, st, sizeof st);
        if (strncmp(st, "sending", 7) != 0) return 1;
        usleep(20000);
    }
    return 0;
}

/* effGetParamDisplay for a param (the message_display / rx_msg_display keys). */
static void dispd(AEffect *a, int idx, char *buf, size_t n) {
    buf[0] = 0;
    a->d(a, 7, idx, 0, buf, 0);
    buf[n - 1] = 0;
}

/* One press of a momentary key (SEND, CLEAR, a stepper's arrow, ...). The wrapper
 * debounces momentary params for 250 ms (one MPC tap reaches it as two presses
 * 30-140 ms apart; vst.json debounce_ms), and a second press of the SAME key inside
 * that window is swallowed -- a test tapping keys back to back would silently do
 * nothing. Space every press out by waiting here. */
static void tap(AEffect *a, int idx) {
    a->setP(a, idx, 1.0f);
    a->setP(a, idx, 0.0f);
    usleep(300000);
}

/* Force one sweep now instead of waiting out the 2 s period: RESCAN sets
 * need_sweep, the reader sweeps within its 200 ms poll, and the 500 ms here are
 * two more of those (tap()'s own 300 ms debounce wait is on top of that). */
static void sweep_now(AEffect *a, int i_rescan) {
    tap(a, i_rescan);
    usleep(500000);
}

/* Which slot holds this file name. message_display is a scrolling window for a name
 * longer than the MESSAGE box, so the display can't be matched against the whole
 * name: the state chunk's file= carries the raw name of whatever slot is selected. */
static int slot_of(AEffect *a, int i_msg, const char *name) {
    char pat[160], buf[1024];
    snprintf(pat, sizeof pat, "file=%s;", name);
    for (int k = 1; k <= 64; k++) {
        a->setP(a, i_msg, (float)(k - 1) / 63.0f);
        void *ch = 0;
        intptr_t n = a->d(a, 23, 0, 0, &ch, 0);
        if (n <= 0 || !ch || n >= (intptr_t)sizeof buf) continue;
        memcpy(buf, ch, (size_t)n);
        buf[n] = 0;
        if (strstr(buf, pat)) return k;
    }
    return 0;
}

int main(void) {
    remove("build/messages/rx_001.syx");          /* a rerun starts from a clean folder */
    remove("build/messages/rx_002.syx");
    remove("build/messages/a_long_enough_file_name_to_clip_the_box.syx");
    remove("build/messages/zz_paced.syx");
    AEffect *a = VSTPluginMain(host);
    CHECK(a != NULL, "plugin instantiates");
    if (!a) return 1;
    run(a, 10);                                   /* first text/display_rev poll syncs last_norm */

    int i_send = find_param("send"), i_src = find_param("send_src"), i_n = find_param("nbytes");
    int i_msg = find_param("message"), i_prev = find_param("msg_prev"), i_next = find_param("msg_next");
    int i_rx = find_param("rx"), i_rxn = find_param("rx_total"), i_tx = find_param("tx");
    int i_tx2 = find_param("tx2"), i_rx2 = find_param("rx2");
    int i_st = find_param("status"), i_save = find_param("save_rx"), i_list = find_param("listen");
    int i_clr = find_param("clear_rx");
    int i_port = find_param("port"), i_b1 = find_param("b1"), i_b2 = find_param("b2"), i_b3 = find_param("b3");
    int i_sto = find_param("send_to"), i_rfrom = find_param("recv_from");
    int i_stp = find_param("send_to_prev"), i_stn = find_param("send_to_next");
    int i_rfp = find_param("recv_from_prev"), i_rfn = find_param("recv_from_next");
    int i_txd = find_param("tx_delay"), i_txb = find_param("tx_buf"), i_txg = find_param("tx_gap");
    int i_rmsg = find_param("rx_msg"), i_rmsgp = find_param("rx_msg_prev"), i_rmsgh = find_param("rx_msg_next");
    int i_rescan = find_param("rescan");
    CHECK(i_send >= 0 && i_src >= 0 && i_n >= 0 && i_msg >= 0 && i_rx >= 0 && i_rxn >= 0 &&
          i_tx >= 0 && i_st >= 0 && i_save >= 0 && i_list >= 0 && i_port >= 0 && i_clr >= 0 &&
          i_prev >= 0 && i_next >= 0 && i_b1 >= 0 && i_b2 >= 0 && i_b3 >= 0 &&
          i_sto >= 0 && i_rfrom >= 0 && i_stp >= 0 && i_stn >= 0 && i_rfp >= 0 && i_rfn >= 0 &&
          i_tx2 >= 0 && i_rx2 >= 0 &&
          i_txd >= 0 && i_txb >= 0 && i_txg >= 0 && i_rescan >= 0 &&
          i_rmsg >= 0 && i_rmsgp >= 0 && i_rmsgh >= 0,
          "every loopback key is a host param");
    CHECK(a->getP(a, i_list) < 1e-3f, "LISTEN defaults to OFF");
    CHECK(a->getP(a, i_sto) < 1e-3f && a->getP(a, i_rfrom) < 1e-3f,
          "OUT PORT and IN PORT default to None (%.0f, %.0f)", a->getP(a, i_sto), a->getP(a, i_rfrom));
    CHECK(fabsf(a->getP(a, i_txd) - 10.0f / 1000.0f) < 1e-4f &&
          fabsf(a->getP(a, i_txb) - (512.0f - 64.0f) / 4032.0f) < 1e-4f &&
          fabsf(a->getP(a, i_txg) - 1.0f / 1000.0f) < 1e-4f,
          "pacing defaults are gentle: 10 ms / 512 B / 1 ms (%.4f, %.4f, %.4f)",
          a->getP(a, i_txd), a->getP(a, i_txb), a->getP(a, i_txg));
    {   char md[64];
        dispd(a, i_rmsg, md, sizeof md);
        CHECK(!strcmp(md, "-"), "RX MSG reads - on an empty capture (\"%s\")", md);
    }
    a->setP(a, i_list, 1.0f);                    /* receiving is opt-in now */

    /* -- the plugin's own ports, read from its PORT readout ---------------- */
    char ptxt[256];
    disp(a, i_port, ptxt, sizeof ptxt);
    int pc = -1, pout = -1, pin = -1;
    sscanf(ptxt, "Sysex Bandit c%d out%d in%d", &pc, &pout, &pin);
    CHECK(pc > 0 && pout >= 0 && pin >= 0, "PORT readout names the ports (%s)", ptxt);
    if (!(pc > 0 && pout >= 0 && pin >= 0)) return 1;

    /* -- probe: "Probe A" (our source and the receiver we read), plus a second
     * receiver ("Probe B") and a second source ("Probe C") so each picker can be
     * shown to choose one and leave the other alone --------------------- */
    CHECK(snd_seq_open(&pseq, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK) >= 0, "probe opens the sequencer");
    snd_seq_set_client_name(pseq, "Probe A");
    p_out = snd_seq_create_simple_port(pseq, "Out",
        SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    int p_in = snd_seq_create_simple_port(pseq, "In",
        SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    CHECK(p_out >= 0 && p_in >= 0, "probe ports out=%d in=%d", p_out, p_in);

    CHECK(snd_seq_open(&pseq_b, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK) >= 0, "second receiver opens");
    snd_seq_set_client_name(pseq_b, "Probe B");
    int p_in_b = snd_seq_create_simple_port(pseq_b, "In",
        SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    CHECK(p_in_b >= 0, "second receiver's in-port = %d", p_in_b);

    CHECK(snd_seq_open(&pseq_c, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK) >= 0, "second sender opens");
    snd_seq_set_client_name(pseq_c, "Probe C");
    p_out_c = snd_seq_create_simple_port(pseq_c, "Out",
        SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    CHECK(p_out_c >= 0, "second sender's out-port = %d", p_out_c);

    /* Nothing is wired yet: OUT PORT and IN PORT are both None by default, so a
     * fresh plugin neither sends nor hears anything until a port is picked. */
    msg_t got[8], gotb[8];
    char t[256], st[256];
    int n;

    a->setP(a, i_src, 1.0f);                       /* BYTES */
    tap(a, i_send);
    n = probe_drain(got, 8);
    disp(a, i_st, st, sizeof st);
    disp(a, i_tx, t, sizeof t);
    CHECK(n == 0 && strstr(st, "pick OUT PORT") != NULL,
          "SEND with OUT PORT = None goes nowhere (%d, \"%s\")", n, st);
    CHECK(!strcmp(t, "-"), "TX shows the dash while OUT PORT is None (\"%s\")", t);

    /* pick the probes: IN PORT to hear them, OUT PORT to reach them */
    int kIn = find_tg_wait(a, i_rfrom, 64, "Probe A:Out");
    int kOut = find_tg_wait(a, i_sto, 64, "Probe A:In");
    CHECK(kIn > 0 && kOut > 0,
          "the pickers list both ends of the probe (in=%d out=%d)", kIn, kOut);
    a->setP(a, i_rfrom, (float)kIn / 64.0f);
    a->setP(a, i_sto, (float)kOut / 64.0f);

    /* -- IN PORT picked: the plugin's sweep must find US (nothing the other
     *    way round: with a port chosen it addresses us directly) ------------ */
    CHECK(wait_swept(a, i_rxn), "IN PORT picked: the plugin subscribes and counts");

    static const uint8_t ident[]  = {0xF0, 0x7E, 0x7F, 0x06, 0x01, 0xF7};
    static const uint8_t custom[] = {0xF0, 0x41, 0x44};   /* nbytes=3: no F7, by design */
    static const uint8_t gm[]     = {0xF0, 0x7E, 0x7F, 0x09, 0x01, 0xF7};
    static const uint8_t gs[]     = {0xF0, 0x41, 0x10, 0x41, 0x40, 0x7F, 0x00, 0x41, 0xF7};
    static const uint8_t inj[]    = {0xF0, 0x11, 0x22, 0x33, 0xF7};
    static const uint8_t later[]  = {0xF0, 0x45, 0x66, 0x77, 0xF7};

    /* -- SEND in BYTES mode: the identity request from b1..b8 defaults ----- */
    a->setP(a, i_src, 1.0f);                       /* BYTES */
    tap(a, i_send);
    CHECK(wait_sent(a, i_st), "the identity request leaves the queue");
    n = probe_drain(got, 8);
    disp(a, i_tx, t, sizeof t); disp(a, i_st, st, sizeof st);
    CHECK(n == 1 && is(&got[0], ident, sizeof ident),
          "SEND/BYTES delivers the identity request (%d event%s, tx \"%s\")", n, n == 1 ? "" : "s", t);
    CHECK(!strcmp(t, "F0 7E 7F 06 01 F7") && strstr(st, "sent 6 B"),
          "TX/STATUS show it (\"%s\", \"%s\")", t, st);
    {   char t2[256];
        disp(a, i_tx2, t2, sizeof t2);
        CHECK(t2[0] == 0, "TX line 2 is empty for a 6-byte message (\"%s\")", t2);
    }

    /* -- three hand-edited bytes: F0 41 44 --------------------------------- */
    a->setP(a, i_n, 2.0f / 7.0f);                  /* nbytes = 3 */
    a->setP(a, i_b1, 240.0f / 255.0f);             /* F0 */
    a->setP(a, i_b2, 65.0f / 255.0f);              /* 41 */
    a->setP(a, i_b3, 68.0f / 255.0f);              /* 44 */
    tap(a, i_send);
    wait_sent(a, i_st);
    n = probe_drain(got, 8);
    {   char hx[80] = ""; int o = 0;
        for (int k = 0; k < n && k < 1; k++)
            for (int j = 0; j < got[k].len && o < 60; j++) o += sprintf(hx + o, "%s%02X", j ? " " : "", got[k].b[j]);
        CHECK(n == 1 && is(&got[0], custom, sizeof custom),
              "SEND/BYTES with edited bytes delivers F0 41 44 (%d event%s, got \"%s\", len %d/%zu)",
              n, n == 1 ? "" : "s", hx, n > 0 ? got[0].len : -1, sizeof custom);
    }

    /* -- SEND in FILE mode, then a two-message file ------------------------ */
    a->setP(a, i_src, 0.0f);                       /* FILE */
    a->setP(a, i_msg, 0.0f);                       /* slot 1: 01_identity_request.syx */
    tap(a, i_send);
    wait_sent(a, i_st);
    n = probe_drain(got, 8);
    CHECK(n == 1 && is(&got[0], ident, sizeof ident),
          "SEND/FILE slot 1 delivers 01_identity_request.syx (%d event%s)", n, n == 1 ? "" : "s");

    a->setP(a, i_msg, 6.0f / 63.0f);               /* slot 7: 07_all_resets.syx */
    tap(a, i_send);
    wait_sent(a, i_st);
    n = probe_drain(got, 8);
    disp(a, i_tx, t, sizeof t);
    CHECK(n == 2 && is(&got[0], gm, sizeof gm) && is(&got[1], gs, sizeof gs),
          "a two-message file splits and sends both in order (%d events)", n);
    CHECK(!strncmp(t, "F0 7E 7F 09 01 F7", 17) && strstr(t, "F0 41 10") != NULL,
          "TX shows the whole selected file, not just its first message (\"%s\")", t);

    /* -- OUT PORT: aim SEND at one receiver, not every one ------------------ */
    a->setP(a, i_src, 1.0f);                       /* BYTES: exactly one message per SEND */
    probe_drain(got, 8); probe_b_drain(gotb, 8);   /* flush what the FILE sends left behind */
    {   int kA = find_tg(a, i_sto, 64, "Probe A:In");
        int kB = find_tg(a, i_sto, 64, "Probe B:In");
        CHECK(kA > 0 && kB > 0 && kA != kB, "OUT PORT lists both receivers (A=%d, B=%d)", kA, kB);
        if (kA > 0 && kB > 0) {
            a->setP(a, i_sto, (float)kA / 64.0f);
            tap(a, i_send);
            wait_sent(a, i_st);
            n = probe_drain(got, 8); int m = probe_b_drain(gotb, 8);
            CHECK(count_to(got, n, p_in) == 1 && m == 0,
                  "OUT PORT = A delivers to A and to nobody else (%d, %d)", n, m);

            a->setP(a, i_sto, (float)kB / 64.0f);
            tap(a, i_send);
            wait_sent(a, i_st);
            n = probe_drain(got, 8); m = probe_b_drain(gotb, 8);
            CHECK(n == 0 && count_to(gotb, m, p_in_b) == 1,
                  "OUT PORT = B delivers to B and to nobody else (%d, %d)", n, m);

            a->setP(a, i_sto, 0.0f);               /* None: no destination at all */
            tap(a, i_send);
            n = probe_drain(got, 8); m = probe_b_drain(gotb, 8);
            disp(a, i_st, st, sizeof st);
            CHECK(n == 0 && m == 0 && strstr(st, "pick OUT PORT") != NULL,
                  "OUT PORT = None sends to nobody (%d, %d, \"%s\")", n, m, st);

            int last = last_tg(a, i_sto, 64);
            if (last < 64) {                      /* one past the last port that exists */
                a->setP(a, i_sto, (float)(last + 1) / 64.0f);
                tap(a, i_send);
                n = probe_drain(got, 8); m = probe_b_drain(gotb, 8);
                disp(a, i_st, st, sizeof st);
                CHECK(n == 0 && m == 0 && strstr(st, "not found") != NULL,
                      "a vanished OUT PORT refuses instead of broadcasting (%d, %d, \"%s\")",
                      n, m, st);
            } else printf("skip a vanished OUT PORT (64 ports are plugged in)\n");
            a->setP(a, i_sto, 0.0f);
        }
    }
    tap(a, i_stn);      /* the stepper's next arrow */
    CHECK(fabsf(a->getP(a, i_sto) - 1.0f / 64.0f) < 1e-4f,
          "OUT PORT NEXT steps None -> 1 (%.4f)", a->getP(a, i_sto));
    tap(a, i_stp);      /* and prev */
    CHECK(a->getP(a, i_sto) < 1e-3f, "OUT PORT PREV steps back to None (%.4f)", a->getP(a, i_sto));
    a->setP(a, i_sto, (float)kOut / 64.0f);        /* aim it back at the probe */
    a->setP(a, i_src, 0.0f);                       /* FILE again for the tests below */

    /* -- receive: inject, RX TOTAL moves, display_rev reports it ----------- */
    tap(a, i_clr);                           /* the capture starts empty: only the injection */
    memset(automated, 0, sizeof automated);
    float rxn_before = a->getP(a, i_rxn);
    CHECK(probe_send(inj, sizeof inj) >= 0, "probe injects SysEx into SysEx In");
    {   /* the reader polls every 200 ms; give it a moment, then let housekeeping run */
        int saw = 0;
        for (int k = 0; k < 25 && !saw; k++) { usleep(40000); saw = a->getP(a, i_rxn) > rxn_before; }
        CHECK(saw, "RX TOTAL counted the injection (%.0f -> %.0f)", rxn_before, a->getP(a, i_rxn));
    }
    disp(a, i_rx, t, sizeof t); disp(a, i_st, st, sizeof st);
    CHECK(!strcmp(t, "F0 11 22 33 F7") && strstr(st, "rx 5 B") && strstr(st, "from Probe A"),
          "RX/STATUS show the message and who sent it (\"%s\", \"%s\")", t, st);
    run(a, 150);                                   /* >= one 4410-frame display_rev poll */
    CHECK(automated[i_rxn] > 0, "RX TOTAL's change was reported to the host (%d time%s)",
          automated[i_rxn], automated[i_rxn] == 1 ? "" : "s");

    /* -- echo filter: our own bytes coming back are dropped ---------------- */
    a->setP(a, i_msg, 0.0f);                       /* re-send slot 1 first: the filter's
                                                    * window runs from the send, and this
                                                    * suite's own send of it was seconds
                                                    * ago */
    (void)a->getP(a, i_msg);                       /* a set of param 0 waits for a flush */
    tap(a, i_send);
    CHECK(wait_sent(a, i_st), "the echo fixture leaves the queue again");
    probe_drain(got, 8);                           /* the copy the probe received */
    rxn_before = a->getP(a, i_rxn);
    probe_send(ident, sizeof ident);               /* the same bytes coming straight back */
    usleep(400000);
    CHECK(a->getP(a, i_rxn) == rxn_before, "the echo filter drops our own message coming back");

    /* -- SAVE RX writes the file and the slot list picks it up -------------- */
    tap(a, i_save);
    disp(a, i_st, st, sizeof st);
    CHECK(strstr(st, "saved rx_001.syx") != NULL, "SAVE RX writes a file (%s)", st);
    {   /* the file holds exactly what was received */
        FILE *f = fopen("build/messages/rx_001.syx", "rb");
        uint8_t buf[16];
        size_t gotn = f ? fread(buf, 1, sizeof buf, f) : 0;
        if (f) fclose(f);
        CHECK(gotn == sizeof inj && !memcmp(buf, inj, sizeof inj),
              "rx_001.syx contains the received bytes (%zu)", gotn);
    }
    a->setP(a, i_msg, 7.0f / 63.0f);               /* slot 8 after the re-scan */
    {   /* effGetParamDisplay returns <key>_display: the file name */
        char md[256];
        md[0] = 0;
        a->d(a, 7, i_msg, 0, md, 0); md[sizeof md - 1] = 0;
        CHECK(!strcmp(md, "rx_001.syx"), "the saved file became a message slot (\"%s\")", md);
    }

    /* -- reassembly: a dump arriving in pieces counts as ONE message ------- */
    static const uint8_t fr1[] = {0xF0, 0x42, 0x43};        /* first fragment, no F7 yet */
    static const uint8_t fr2[] = {0x44, 0x45, 0xF7};        /* the tail */
    rxn_before = a->getP(a, i_rxn);
    CHECK(probe_send(fr1, sizeof fr1) >= 0 && probe_send(fr2, sizeof fr2) >= 0,
          "probe injects a message as two fragments");
    {   int saw = 0;
        for (int k = 0; k < 25 && !saw; k++) { usleep(40000); saw = a->getP(a, i_rxn) > rxn_before; }
        /* exactly one message: the normalized step of rx_total (0..999) is 1/999 */
        CHECK(saw && fabsf((a->getP(a, i_rxn) - rxn_before) - 1.0f / 999.0f) < 1e-4f,
              "the two fragments count as ONE message (%.5f)", a->getP(a, i_rxn) - rxn_before);
    }
    disp(a, i_rx, t, sizeof t); disp(a, i_st, st, sizeof st);
    CHECK(!strcmp(t, "F0 42 43 44 45 F7") && strstr(st, "rx 6 B"),
          "RX/STATUS show the whole reassembled message (\"%s\", \"%s\")", t, st);

    /* -- an aborted partial never prefixes the next message: a new F0 restarts
     *    the stream (a looped-back BYTES fragment has no F7 of its own) ------ */
    static const uint8_t half[] = {0xF0, 0x41, 0x44};      /* unterminated, like nbytes=3 */
    static const uint8_t real[] = {0xF0, 0x21, 0x22, 0xF7};
    rxn_before = a->getP(a, i_rxn);
    CHECK(probe_send(half, sizeof half) >= 0, "probe injects an unterminated fragment");
    usleep(300000);
    CHECK(probe_send(real, sizeof real) >= 0 && rx_seen(a, i_rxn, rxn_before),
          "a message after it is still counted");
    disp(a, i_rx, t, sizeof t);
    CHECK(!strcmp(t, "F0 21 22 F7"),
          "the dead fragment is dropped, not prefixed (\"%s\")", t);

    /* -- CLEAR resets the received side ------------------------------------ */
    tap(a, i_clr);
    disp(a, i_rx, t, sizeof t); disp(a, i_st, st, sizeof st);
    CHECK(a->getP(a, i_rxn) < 1e-3f && !strcmp(t, "-") && strstr(st, "cleared"),
          "CLEAR zeroes RX TOTAL, empties RX, says so (%.3f, \"%s\", \"%s\")",
          a->getP(a, i_rxn), t, st);

    /* -- ... but the echo window expires: the same bytes arriving later are a
     *    real message (a read-back of the patch you sent), not our own echo
     *    forever -- without this, a round-trip test loses its replies -------- */
    rxn_before = a->getP(a, i_rxn);
    usleep(5500000);                              /* well past ECHO_MS since that send */
    CHECK(probe_send(ident, sizeof ident) >= 0 && rx_seen(a, i_rxn, rxn_before),
          "after the echo window the same bytes count again (%.3f -> %.3f)",
          rxn_before, a->getP(a, i_rxn));

    /* -- LISTEN OFF stops receiving, ON sweeps again ------------------------ */
    a->setP(a, i_list, 0.0f);                      /* OFF */
    disp(a, i_st, st, sizeof st);
    CHECK(strstr(st, "listen off") != NULL, "LISTEN OFF (%s)", st);
    rxn_before = a->getP(a, i_rxn);
    probe_send(later, sizeof later);
    usleep(500000);
    CHECK(a->getP(a, i_rxn) == rxn_before, "with LISTEN OFF the injection goes nowhere");
    a->setP(a, i_list, 1.0f);                      /* ON */
    CHECK(wait_swept(a, i_rxn), "LISTEN ON sweeps and receives again");

    /* -- IN PORT: hear the one source picked, not the whole box ------------ */
    static const uint8_t only_a[] = {0xF0, 0x53, 0x54, 0xF7};
    static const uint8_t only_c[] = {0xF0, 0x55, 0x56, 0xF7};
    static const uint8_t none_a[] = {0xF0, 0x59, 0x5A, 0xF7};
    static const uint8_t none_c[] = {0xF0, 0x5B, 0x5C, 0xF7};
    {   int kA = find_tg(a, i_rfrom, 64, "Probe A:Out");
        int kC = find_tg(a, i_rfrom, 64, "Probe C:Out");
        CHECK(kA > 0 && kC > 0 && kA != kC, "IN PORT lists both sources (A=%d, C=%d)", kA, kC);
        usleep(700000);                           /* the two scans settled back on the pick */

        rxn_before = a->getP(a, i_rxn);
        probe_c_send(only_c, sizeof only_c);
        usleep(700000);
        CHECK(a->getP(a, i_rxn) == rxn_before,
              "IN PORT = A does not hear C (%.0f -> %.0f)", rxn_before, a->getP(a, i_rxn));

        if (kC > 0) {
            a->setP(a, i_rfrom, (float)kC / 64.0f);
            usleep(700000);
            rxn_before = a->getP(a, i_rxn);
            CHECK(probe_c_send(only_c, sizeof only_c) >= 0 && rx_seen(a, i_rxn, rxn_before),
                  "IN PORT = C hears C");
        }

        a->setP(a, i_rfrom, 0.0f);               /* None: nobody at all */
        usleep(700000);
        rxn_before = a->getP(a, i_rxn);
        probe_send(none_a, sizeof none_a);
        probe_c_send(none_c, sizeof none_c);
        usleep(700000);
        CHECK(a->getP(a, i_rxn) == rxn_before,
              "IN PORT = None hears nobody (%.0f -> %.0f)", rxn_before, a->getP(a, i_rxn));

        if (kA > 0) a->setP(a, i_rfrom, (float)kA / 64.0f);   /* back to the suite's pick */
        usleep(700000);
        rxn_before = a->getP(a, i_rxn);
        CHECK(probe_send(only_a, sizeof only_a) >= 0 && rx_seen(a, i_rxn, rxn_before),
              "IN PORT = A hears A again");
    }

    /* -- the pick follows the PORT, not its place in the list --------------- */
    {
        /* The lists are rebuilt in ALSA's discovery order every sweep, so one
         * client coming or going shifts every index after it: a pick remembered
         * only as an index silently comes to mean a different device, or to
         * nothing at all. The pick is remembered by identity instead -- client:port
         * first, the "client:port" name for a device that comes back with a new
         * id -- and (gone) is what a vanished one reads. */
        char d[256];
        a->setP(a, i_src, 1.0f);                       /* BYTES: one message per SEND */

        /* IN: take the picked source's port away. The index it sat at now holds
         * the next source (Probe C), so a positional pick would slide onto C. */
        sweep_now(a, i_rescan);
        int old_out = p_out;
        rxn_before = a->getP(a, i_rxn);
        snd_seq_delete_port(pseq, p_out);              /* Probe A:Out leaves the list */
        p_out = -1;
        sweep_now(a, i_rescan);
        disp(a, i_rfrom, d, sizeof d);
        CHECK(strstr(d, "(gone)") != NULL,
              "a vanished IN PORT reads (gone), not the next source (\"%s\")", d);
        probe_c_send(only_c, sizeof only_c);
        usleep(700000);
        CHECK(a->getP(a, i_rxn) == rxn_before,
              "the pick does not slide onto the next source (%.0f -> %.0f)",
              rxn_before, a->getP(a, i_rxn));

        /* ... and the source comes back: same name, new port id (a temp port
         * holds the old id first, so re-finding it has to go by name) */
        {   int caps = SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ;
            int typ = SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION;
            int tmp = snd_seq_create_simple_port(pseq, "Out", caps, typ);
            p_out = snd_seq_create_simple_port(pseq, "Out", caps, typ);
            if (tmp >= 0) snd_seq_delete_port(pseq, tmp);
            CHECK(p_out >= 0 && p_out != old_out,
                  "Probe A's out-port comes back on a new id (%d -> %d)", old_out, p_out);
        }
        CHECK(wait_swept(a, i_rxn), "IN PORT re-finds the source by name after its port id changed");
        disp(a, i_rfrom, d, sizeof d);
        CHECK(strstr(d, "Probe A:Out") != NULL, "the readout names it again (\"%s\")", d);

        /* OUT: the same story with a destination. Probe B is picked; taking
         * Probe A's in-port away moves every later index down one. */
        int kB = find_tg_wait(a, i_sto, 64, "Probe B:In");
        CHECK(kB > 0, "OUT PORT still lists Probe B (%d)", kB);
        if (kB > 0) {
            a->setP(a, i_sto, (float)kB / 64.0f);
            sweep_now(a, i_rescan);
            snd_seq_delete_port(pseq, p_in);           /* Probe A:In leaves the list */
            sweep_now(a, i_rescan);
            disp(a, i_sto, d, sizeof d);
            CHECK(strstr(d, "Probe B:In") != NULL,
                  "OUT PORT follows its device across a shift (\"%s\")", d);
            tap(a, i_send);
            CHECK(wait_sent(a, i_st), "SEND after the shift leaves the queue");
            n = probe_drain(got, 8);
            {   int m = probe_b_drain(gotb, 8);
                CHECK(n == 0 && count_to(gotb, m, p_in_b) == 1,
                      "SEND still reaches B after the list shifted (%d, %d)", n, m);
            }

            /* B goes away too: (gone) and a refusal, never a broadcast */
            snd_seq_delete_port(pseq_b, p_in_b);
            sweep_now(a, i_rescan);
            disp(a, i_sto, d, sizeof d);
            CHECK(strstr(d, "(gone)") != NULL, "a vanished OUT PORT reads (gone) (\"%s\")", d);
            tap(a, i_send);
            n = probe_drain(got, 8);
            {   int m = probe_b_drain(gotb, 8);
                disp(a, i_st, st, sizeof st);
                CHECK(n == 0 && m == 0 && strstr(st, "not found") != NULL,
                      "SEND to a vanished port refuses (%d, %d, \"%s\")", n, m, st);
            }

            /* B comes back under a new port id: found by name again (the temp
             * port holds the old id first, as above) */
            {   int caps = SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE;
                int typ = SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION;
                int tmp = snd_seq_create_simple_port(pseq_b, "In", caps, typ);
                p_in_b = snd_seq_create_simple_port(pseq_b, "In", caps, typ);
                if (tmp >= 0) snd_seq_delete_port(pseq_b, tmp);
                CHECK(p_in_b >= 0, "Probe B's in-port comes back (id %d)", p_in_b);
            }
            p_in = snd_seq_create_simple_port(pseq, "In",
                SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
                SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
            CHECK(p_in >= 0, "Probe A's in-port is recreated (id %d)", p_in);
            sweep_now(a, i_rescan);
            disp(a, i_sto, d, sizeof d);
            CHECK(strstr(d, "Probe B:In") != NULL,
                  "OUT PORT re-finds the device by name (\"%s\")", d);
            tap(a, i_send);
            CHECK(wait_sent(a, i_st), "SEND to the re-found device leaves the queue");
            n = probe_drain(got, 8);
            {   int m = probe_b_drain(gotb, 8);
                CHECK(n == 0 && count_to(gotb, m, p_in_b) == 1,
                      "SEND reaches B again on its new port (%d, %d)", n, m);
            }

            /* restore the suite's standing pick: OUT PORT = Probe A:In */
            a->setP(a, i_sto, 0.0f);
            {   int kA2 = find_tg_wait(a, i_sto, 64, "Probe A:In");
                CHECK(kA2 > 0, "OUT PORT is back on Probe A (%d)", kA2);
                if (kA2 > 0) a->setP(a, i_sto, (float)kA2 / 64.0f);
            }
            sweep_now(a, i_rescan);
        }
        a->setP(a, i_src, 0.0f);                       /* FILE again for the tests below */
    }

    /* -- a bank dump: ALSA cuts SysEx at 4096 bytes per event, so a big message
     *    arrives as a run of fragments and has to be reassembled whole ------ */
    {
        enum { BIG = 64000, FRAG = 1000 };      /* >> the 8 KB this used to drop */
        tap(a, i_clr);   /* capture = this dump alone */
        static uint8_t big[BIG];
        big[0] = 0xF0;
        for (int i = 1; i < BIG - 1; i++) big[i] = (uint8_t)(i & 0x7F);   /* legal data bytes */
        big[BIG - 1] = 0xF7;
        rxn_before = a->getP(a, i_rxn);
        int ok = 1, nfrag = 0, err = 0;
        for (int off = 0; off < BIG && ok; off += FRAG) {
            int n = BIG - off < FRAG ? BIG - off : FRAG;
            int r = probe_send(big + off, n);
            if (r < 0) { ok = 0; if (!err) err = r; }
            nfrag++;
        }
        CHECK(ok && nfrag == 64, "probe injects a %d-byte dump as %d fragments (first ret %d)",
              BIG, nfrag, err);
        CHECK(rx_seen(a, i_rxn, rxn_before), "the whole bank dump counts as one message");
        char t2[256];
        disp(a, i_rx, t, sizeof t);
        CHECK(!strncmp(t, "F0 01 02 03 04", 14), "RX shows the head of the dump (\"%s\")", t);
        disp(a, i_rx2, t2, sizeof t2);
        CHECK(!strncmp(t2, "1B", 2),
              "RX line 2 continues where line 1 stopped (rx=%d/%s rx2=%d/%s -> \"%s\")",
              i_rx, PARAMS[i_rx].key, i_rx2, PARAMS[i_rx2].key, t2);
        tap(a, i_save);
        disp(a, i_st, st, sizeof st);
        {   FILE *f = fopen("build/messages/rx_002.syx", "rb");
            uint8_t *b = malloc(BIG + 1);
            size_t gotn = f ? fread(b, 1, BIG + 1, f) : 0;
            if (f) fclose(f);
            int whole = gotn == (size_t)BIG && b[0] == 0xF0 && b[BIG - 1] == 0xF7 &&
                        !memcmp(b, big, (size_t)BIG);
            CHECK(whole, "SAVE RX wrote all %d bytes back (%zu, \"%s\")", BIG, gotn, st);
            free(b);
        }
        remove("build/messages/rx_002.syx");
    }

    /* -- 1.0.1: the capture keeps EVERY message, RX MSG steps through it, the
     *    SPEED knobs pace a send, and a long file name scrolls ------------- */
    {
        /* two fixtures: a name too long for the 140 px MESSAGE box, and a message
         * big enough to be split into several tx_buf blocks */
        static const char LONGN[] = "a_long_enough_file_name_to_clip_the_box.syx";
        static uint8_t paced[500];
        paced[0] = 0xF0;
        for (int i = 1; i < (int)sizeof paced - 1; i++) paced[i] = (uint8_t)(i & 0x7F);
        paced[sizeof paced - 1] = 0xF7;
        {   FILE *f = fopen("build/messages/a_long_enough_file_name_to_clip_the_box.syx", "wb");
            CHECK(f != NULL, "the long-named fixture is written");
            if (f) { fputc(0xF0, f); fputc(0xF7, f); fclose(f); }
        }
        {   FILE *f = fopen("build/messages/zz_paced.syx", "wb");
            size_t w = f ? fwrite(paced, 1, sizeof paced, f) : 0;
            if (f) fclose(f);
            CHECK(w == sizeof paced, "the paced fixture is written (%zu B)", w);
        }
        tap(a, i_rescan);
        int sLong = slot_of(a, i_msg, LONGN), sPaced = slot_of(a, i_msg, "zz_paced.syx");
        CHECK(sLong > 0 && sPaced > 0, "RESCAN lists both fixtures (long=%d, paced=%d)", sLong, sPaced);

        /* -- a file name too long for the box scrolls through it ------------- */
        if (sLong) {
            char w1[64], w2[64];
            a->setP(a, i_msg, (float)(sLong - 1) / 63.0f);
            dispd(a, i_msg, w1, sizeof w1);
            run(a, 240);                            /* several display_rev polls */
            dispd(a, i_msg, w2, sizeof w2);
            CHECK(strlen(w1) == 13 && strcmp(w1, w2) != 0,
                  "the long name scrolls in the MESSAGE box (\"%s\" -> \"%s\")", w1, w2);

            /* -- selecting a file restarts its scroll at the head ------------ */
            char head[16], one[16], w3[64];
            memcpy(head, LONGN, 13); head[13] = 0;      /* the window at phase 0 */
            memcpy(one, LONGN + 1, 13); one[13] = 0;    /* and at phase 1, in case a
                                                         * display_rev poll slipped
                                                         * in between set and read */
            a->setP(a, i_msg, 0.0f);                    /* another file ... */
            (void)a->getP(a, i_msg);                    /* ... applied: a set of param 0
                                                         * is held until a flush (the
                                                         * wrapper's pend0), and a pair
                                                         * ending where it started is
                                                         * dropped */
            a->setP(a, i_msg, (float)(sLong - 1) / 63.0f);   /* ... and back again */
            dispd(a, i_msg, w3, sizeof w3);             /* the display read flushes it */
            CHECK(!strcmp(w3, head) || !strcmp(w3, one),
                  "reselecting a file restarts its scroll at the head (\"%s\" vs \"%s\")", w3, head);
        }

        /* -- the JD-990 case: a bank that is MANY messages is kept whole ----- */
        static const uint8_t blk[5][6] = {
            {0xF0, 0x41, 0x10, 0x20, 0x01, 0xF7}, {0xF0, 0x41, 0x10, 0x21, 0x02, 0xF7},
            {0xF0, 0x41, 0x10, 0x22, 0x03, 0xF7}, {0xF0, 0x41, 0x10, 0x23, 0x04, 0xF7},
            {0xF0, 0x41, 0x10, 0x24, 0x05, 0xF7} };
        static const uint8_t blk6[] = {0xF0, 0x41, 0x10, 0x25, 0x06, 0xF7};
        char md[64];
        tap(a, i_clr);
        rxn_before = a->getP(a, i_rxn);
        for (int k = 0; k < 5; k++) probe_send(blk[k], sizeof blk[0]);
        {   int saw = 0;
            for (int k = 0; k < 40 && !saw; k++) {
                usleep(40000);
                saw = a->getP(a, i_rxn) - rxn_before >= 5.0f / 999.0f - 1e-4f;
            }
            CHECK(saw, "all five blocks are counted (%.3f -> %.3f)", rxn_before, a->getP(a, i_rxn));
        }
        dispd(a, i_rmsg, md, sizeof md);
        CHECK(!strcmp(md, "5/5"), "RX MSG shows 5 of 5 captured messages (\"%s\")", md);
        disp(a, i_rx, t, sizeof t);
        CHECK(!strcmp(t, "F0 41 10 24 05 F7"), "RX shows the newest block (\"%s\")", t);
        tap(a, i_rmsgp);
        tap(a, i_rmsgp);
        dispd(a, i_rmsg, md, sizeof md); disp(a, i_rx, t, sizeof t);
        CHECK(!strcmp(md, "3/5") && !strcmp(t, "F0 41 10 22 03 F7"),
              "RX MSG PREV steps back through the capture (\"%s\", \"%s\")", md, t);
        tap(a, i_rmsgh);
        dispd(a, i_rmsg, md, sizeof md);
        CHECK(!strcmp(md, "4/5"), "RX MSG NEXT steps forward (\"%s\")", md);
        rxn_before = a->getP(a, i_rxn);
        probe_send(blk6, sizeof blk6);
        CHECK(rx_seen(a, i_rxn, rxn_before), "the sixth block is counted");
        dispd(a, i_rmsg, md, sizeof md);
        CHECK(!strcmp(md, "6/6"), "a new message moves RX MSG to the end (\"%s\")", md);

        tap(a, i_save);
        disp(a, i_st, st, sizeof st);
        CHECK(strstr(st, "saved rx_002.syx") != NULL, "SAVE RX names the file (%s)", st);
        {   FILE *f = fopen("build/messages/rx_002.syx", "rb");
            uint8_t b[64];
            size_t gotn = f ? fread(b, 1, sizeof b, f) : 0;
            if (f) fclose(f);
            CHECK(gotn == 36 && !memcmp(b, blk, 30) && !memcmp(b + 30, blk6, 6),
                  "the whole capture is written, all six blocks in order (%zu B)", gotn);
        }
        remove("build/messages/rx_002.syx");
        /* the save re-scanned the folder with rx_002 in it, so the slots shifted:
         * re-scan without it and re-find the paced fixture */
        tap(a, i_rescan);
        sPaced = slot_of(a, i_msg, "zz_paced.syx");

        /* -- SPEED: 64-byte blocks with a 20 ms gap, byte-exact and slower ---- */
        if (sPaced) {
            a->setP(a, i_txb, 0.0f);                       /* tx_buf = 64 B */
            a->setP(a, i_txg, 20.0f / 1000.0f);            /* tx_gap = 20 ms */
            a->setP(a, i_txd, 0.0f);                       /* tx_delay = 0 ms */
            a->setP(a, i_msg, (float)(sPaced - 1) / 63.0f);
            probe_drain(got, 8);                           /* flush what the capture left */
            struct timespec t0, t1;
            clock_gettime(CLOCK_MONOTONIC, &t0);
            /* manual press, not tap(): the last SEND was seconds ago (not debounced),
             * and tap()'s debounce wait would be measured as pacing */
            a->setP(a, i_send, 1.0f); a->setP(a, i_send, 0.0f);
            int ok = wait_sent(a, i_st);
            clock_gettime(CLOCK_MONOTONIC, &t1);
            long ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_nsec - t0.tv_nsec) / 1000000;
            n = probe_drain(got, 8);
            disp(a, i_st, st, sizeof st);
            CHECK(ok && strstr(st, "sent 1 msg, 500 B") != NULL,
                  "the paced send reports the whole message (%s)", st);
            {   uint8_t all[512];
                int tot = 0, same = 1;
                for (int k = 0; k < n && tot + got[k].len <= (int)sizeof all; k++) {
                    memcpy(all + tot, got[k].b, (size_t)got[k].len);
                    tot += got[k].len;
                }
                same = tot == (int)sizeof paced && !memcmp(all, paced, sizeof paced);
                CHECK(n == 8 && same,
                      "%d B arrived as %d %d-byte blocks and reassemble to the file", tot, n, 64);
            }
            CHECK(ms >= 100 && ms < 10000, "the gaps are real: %ld ms for 8 blocks", ms);
            a->setP(a, i_txb, (512.0f - 64.0f) / 4032.0f); /* back to the defaults */
            a->setP(a, i_txg, 1.0f / 1000.0f);
            a->setP(a, i_txd, 10.0f / 1000.0f);
        }
    }

    /* -- MESSAGE NEXT / PREV step the slot --------------------------------- */
    a->setP(a, i_msg, 4.0f / 63.0f);               /* slot 5 */
    tap(a, i_next);
    CHECK(fabsf(a->getP(a, i_msg) - 5.0f / 63.0f) < 1e-4f, "MESSAGE NEXT steps 5 -> 6 (%.3f)", a->getP(a, i_msg));
    tap(a, i_prev);
    CHECK(fabsf(a->getP(a, i_msg) - 4.0f / 63.0f) < 1e-4f, "MESSAGE PREV steps 6 -> 5 (%.3f)", a->getP(a, i_msg));

    /* -- state chunk restores on a fresh instance, file= by name ----------- */
    a->setP(a, i_msg, 6.0f / 63.0f);               /* slot 7: 07_all_resets.syx */
    a->setP(a, i_sto, 3.0f / 64.0f);               /* OUT PORT 3, to see it come back */
    void *ch = 0;
    intptr_t nch = a->d(a, 23, 0, 0, &ch, 0);
    AEffect *b = VSTPluginMain(host);
    CHECK(nch > 0 && b != NULL, "state chunk reads back (%ld bytes), second instance opens", (long)nch);
    if (b && nch > 0) {
        b->d(b, 24, 0, nch, ch, 0);
        /* the name is longer than the MESSAGE box, so its display scrolls: read the
         * restored slot back out of b's own state chunk instead of the window */
        char md[256];
        md[0] = 0;
        {   void *ch2 = 0;
            intptr_t n2 = b->d(b, 23, 0, 0, &ch2, 0);
            if (n2 > 0 && ch2 && n2 < (intptr_t)sizeof md) { memcpy(md, ch2, (size_t)n2); md[n2] = 0; }
        }
        CHECK(fabsf(b->getP(b, i_msg) - 6.0f / 63.0f) < 1e-4f && strstr(md, "file=07_all_resets.syx;") != NULL,
              "chunk restores the slot and resolves file= by name (%.3f, %s)", b->getP(b, i_msg),
              strstr(md, "file=") ? "file= found" : "no file=");
        CHECK(fabsf(b->getP(b, i_sto) - 3.0f / 64.0f) < 1e-4f,
              "chunk restores OUT PORT (%.4f)", b->getP(b, i_sto));
        b->d(b, 1, 0, 0, 0, 0);                    /* effClose */
    }
    a->d(a, 1, 0, 0, 0, 0);
    snd_seq_close(pseq);
    snd_seq_close(pseq_b);
    snd_seq_close(pseq_c);
    remove("build/messages/rx_001.syx");           /* leave the folder as it was */
    remove("build/messages/rx_002.syx");
    remove("build/messages/a_long_enough_file_name_to_clip_the_box.syx");
    remove("build/messages/zz_paced.syx");
    printf("%s\n", fails ? "FAILED" : "PASSED");
    return fails ? 1 : 0;
}
