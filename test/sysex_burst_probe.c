/* Back-to-back send reliability: are messages lost when several SysEx go out in a row?
 * Variants: direct vs. queued+drain, nonblocking vs. blocking receiver, with/without a gap. */
#include <alsa/asoundlib.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int send_direct(snd_seq_t *seq, int port, const unsigned char *buf, int len)
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

static int send_queued(snd_seq_t *seq, int port, const unsigned char *buf, int len)
{
    snd_seq_event_t ev;
    snd_seq_ev_clear(&ev);
    snd_seq_ev_set_source(&ev, port);
    snd_seq_ev_set_subs(&ev);
    ev.type = SND_SEQ_EVENT_SYSEX;
    snd_seq_ev_set_variable(&ev, len, (void *)buf);
    int r = snd_seq_event_output(seq, &ev);
    if (r >= 0) snd_seq_drain_output(seq);
    return r;
}

static int drain_rx(snd_seq_t *seq, int expect, int timeout_ms)
{
    int got = 0;
    for (;;) {
        struct pollfd fds[4];
        if (snd_seq_poll_descriptors(seq, fds, 4, POLLIN) <= 0) break;
        if (poll(fds, 1, timeout_ms) <= 0) break;
        snd_seq_event_t *ev = NULL;
        if (snd_seq_event_input(seq, &ev) < 0) break;
        if (ev->type == SND_SEQ_EVENT_SYSEX) got++;
        snd_seq_free_event(ev);
        if (got >= expect) break;
    }
    return got;
}

static void trial(const char *label, int (*sendfn)(snd_seq_t *, int, const unsigned char *, int),
                  int count, int gap_us, int rx_block)
{
    snd_seq_t *tx = NULL, *rx = NULL;
    unsigned char buf[8];
    int t, r, sent_ok = 0;

    snd_seq_open(&tx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK);
    snd_seq_set_client_name(tx, "SB-TX");
    t = snd_seq_create_simple_port(tx, "SysEx Out", SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
                                   SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    snd_seq_open(&rx, "default", SND_SEQ_OPEN_DUPLEX, rx_block ? 0 : SND_SEQ_NONBLOCK);
    snd_seq_set_client_name(rx, "SB-RX");
    r = snd_seq_create_simple_port(rx, "SysEx In", SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
                                   SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    snd_seq_connect_from(rx, r, snd_seq_client_id(tx), t);

    buf[0] = 0xF0; buf[2] = 0xF7;
    for (int i = 0; i < count; i++) {
        buf[1] = (unsigned char)i;
        if (sendfn(tx, t, buf, 3) >= 0) sent_ok++;
        if (gap_us) usleep(gap_us);
    }
    int got = drain_rx(rx, count, 300);
    printf("%-42s sent %2d/%2d received %2d  %s\n", label, sent_ok, count, got,
           got == count ? "ok" : "LOST");
    snd_seq_disconnect_from(rx, r, snd_seq_client_id(tx), t);
    snd_seq_close(rx);
    snd_seq_close(tx);
}

int main(void)
{
    trial("direct, nonblock rx, no gap", send_direct, 20, 0, 0);
    trial("direct, nonblock rx, 1ms gap", send_direct, 20, 1000, 0);
    trial("direct, blocking rx, no gap", send_direct, 20, 0, 1);
    trial("queued+drain, nonblock rx, no gap", send_queued, 20, 0, 0);
    trial("queued+drain, blocking rx, no gap", send_queued, 20, 0, 1);
    trial("direct, nonblock rx, 5 messages", send_direct, 5, 0, 0);
    trial("direct, nonblock rx, 100 messages", send_direct, 100, 0, 0);
    return 0;
}
