/* x86 loopback proof for Sysex Bandit's transport, against a real ALSA sequencer (/dev/snd/seq).
 * It pins down the exact contract the engine's send and receive paths must follow:
 *
 *   SEND
 *     1. SND_SEQ_EVENT_SYSEX + snd_seq_ev_set_variable + snd_seq_event_output_direct delivers
 *        byte-exact at 6, 100, 129, 300 and 1000 bytes (the kit's poc/midiport.c only proved
 *        fixed 3-byte events);
 *     2. sending with NO subscriber returns >= 0 -- a track may not be patched to the port yet;
 *     3. back-to-back bursts (a multi-message .syx file, a double-tap) lose nothing.
 *
 *   RECEIVE (the reader-thread shape)
 *     4. poll() once for the wake-up, then drain with a zero-timeout snd_seq_event_input loop
 *        until -EAGAIN. NOT one read per poll: alsa-lib pulls a batch from the fd into its own
 *        buffer, so poll() goes quiet while events still wait -- proven in sysex_read_pattern.c,
 *        which is why a naive poll->read-one->poll loop would stall messages behind the next
 *        arriving event.
 *
 * Build:  gcc -I<alsa-dev include> vst/test/alsa_loop.c -o /tmp/alsa_loop <libasound.so.2> -lpthread
 * Run:    /tmp/alsa_loop            (needs /dev/snd/seq; exit 0 = all ok) */
#include <alsa/asoundlib.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int fails;
#define CHECK(c, ...) do { printf("%s ", (c) ? "ok  " : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!(c)) fails++; } while (0)

/* the engine's send: one SysEx event, direct, no buffering round trip */
static int send_sysex(snd_seq_t *seq, int port, const unsigned char *buf, int len)
{
    snd_seq_event_t ev;
    snd_seq_ev_clear(&ev);
    snd_seq_ev_set_source(&ev, port);
    snd_seq_ev_set_subs(&ev);
    snd_seq_ev_set_direct(&ev);
    ev.type = SND_SEQ_EVENT_SYSEX;
    snd_seq_ev_set_variable(&ev, len, (void *)buf);
    return snd_seq_event_output_direct(seq, &ev);
}

/* the engine's reader core: poll once, then drain everything buffered (see header, point 4).
 * Copies each message out in arrival order; returns the number of SysEx messages drained. */
static int drain_sysex(snd_seq_t *seq, unsigned char out[][1024], int *lens, int max, int timeout_ms)
{
    struct pollfd fds[4];
    if (snd_seq_poll_descriptors(seq, fds, 4, POLLIN) <= 0) return -1;
    if (poll(fds, 1, timeout_ms) <= 0) return 0;          /* nothing arrived in time */
    int n = 0;
    for (;;) {
        snd_seq_event_t *ev = NULL;
        int r = snd_seq_event_input(seq, &ev);            /* >= 0 = event, -EAGAIN = drained */
        if (r < 0) break;
        if (ev->type == SND_SEQ_EVENT_SYSEX && n < max) {
            int len = ev->data.ext.len;
            if (len > 1024) len = 1024;
            memcpy(out[n], ev->data.ext.ptr, len);
            lens[n] = len;
            n++;
        }
        snd_seq_free_event(ev);
    }
    return n;
}

int main(void)
{
    snd_seq_t *tx = NULL, *rx = NULL;
    static unsigned char big[1000];
    static unsigned char got[24][1024];
    int lens[24], t, r, n;

    static const unsigned char id6[] = {0xF0, 0x7E, 0x7F, 0x06, 0x01, 0xF7};

    /* --- the two clients the engine opens (duplex, nonblocking) --- */
    CHECK(snd_seq_open(&tx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK) >= 0, "tx: open duplex nonblock");
    snd_seq_set_client_name(tx, "Sysex Bandit TX");
    t = snd_seq_create_simple_port(tx, "SysEx Out",
        SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    CHECK(t >= 0, "tx: SysEx Out port %d", t);

    CHECK(snd_seq_open(&rx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK) >= 0, "rx: open duplex nonblock");
    snd_seq_set_client_name(rx, "Sysex Bandit RX");
    r = snd_seq_create_simple_port(rx, "SysEx In",
        SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    CHECK(r >= 0, "rx: SysEx In port %d", r);

    /* --- 2. no subscriber yet: the send must still be accepted --- */
    n = send_sysex(tx, t, id6, sizeof id6);
    CHECK(n >= 0, "send with no subscriber returns %d (a track may not be patched yet)", n);

    /* --- subscribe the receiver, as a track does with the plugin's output port --- */
    CHECK(snd_seq_connect_from(rx, r, snd_seq_client_id(tx), t) >= 0, "rx: connect_from tx %d:%d",
          snd_seq_client_id(tx), t);

    /* --- 1. byte-exact delivery across sizes (also covers a lone send after the no-subs one) --- */
    static const int sizes[] = {6, 100, 129, 300, 1000};
    for (unsigned k = 0; k < sizeof sizes / sizeof sizes[0]; k++) {
        int len = sizes[k];
        memset(big, 0, sizeof big);
        big[0] = 0xF0; big[len - 1] = 0xF7;
        for (int i = 1; i < len - 1; i++) big[i] = (unsigned char)(i & 0x7F);
        CHECK(send_sysex(tx, t, big, len) >= 0, "send %d-byte SysEx", len);
        n = drain_sysex(rx, got, lens, 8, 1500);
        CHECK(n == 1 && lens[0] == len && !memcmp(got[0], big, len),
              "%d bytes: %s", len, n == 1 && lens[0] == len && !memcmp(got[0], big, len) ? "byte-exact" : "broken");
    }

    /* --- 3. back-to-back burst: a multi-message .syx or a double-tap must not lose messages --- */
    {
        int sent_ok = 0;
        for (int i = 0; i < 20; i++) {
            unsigned char m[3] = {0xF0, (unsigned char)i, 0xF7};
            if (send_sysex(tx, t, m, 3) >= 0) sent_ok++;
        }
        CHECK(sent_ok == 20, "burst: all 20 sends accepted (%d)", sent_ok);
        n = drain_sysex(rx, got, lens, 24, 1500);
        CHECK(n == 20, "burst: all 20 received in one drain (%d)", n);
        int ordered = n == 20;
        for (int i = 0; ordered && i < 20; i++)
            ordered = lens[i] == 3 && got[i][0] == 0xF0 && got[i][1] == (unsigned char)i && got[i][2] == 0xF7;
        CHECK(ordered, "burst: every message arrived intact, in order");
    }

    /* --- a second drain round: nothing stale is left behind --- */
    CHECK(drain_sysex(rx, got, lens, 24, 300) == 0, "queue empty after the drain");

    snd_seq_disconnect_from(rx, r, snd_seq_client_id(tx), t);
    snd_seq_close(rx);
    snd_seq_close(tx);
    printf("%s\n", fails ? "FAILED" : "PASSED");
    return fails ? 1 : 0;
}
