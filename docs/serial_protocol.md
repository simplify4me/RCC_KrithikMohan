# RC-Car Serial Wire Protocol (v1)

Status: **v1 — frozen for T22 implementation**
Audience: the Pi-side `serial_client` implementer (this repo, T22) and the
out-of-repo Arduino-class MCU firmware implementer.

This document is the single source of truth for the wire format exchanged
between the Raspberry Pi (obstacle-detection host) and the MCU (motor/steer
controller) over a serial link (UART, typically USB-serial, 8N1). Anything
not specified here is out of scope for v1 and must not be assumed by either
side.

---

## 1. Transport assumptions

- Serial link, 8 data bits, no parity, 1 stop bit (8N1).
- Baud rate is a deployment-time config value (see `config/serial.yaml`),
  not part of this spec — both ends must be configured to match out of band.
- All bytes are US-ASCII. No binary framing, no checksums, no CRC in v1.
- Every message is a single line: ASCII fields separated by `,` (comma,
  0x2C), terminated by `\n` (LF, 0x0A). No `\r` (CR) is sent or expected.

---

## 2. Message types

### 2.1 Version header — `V,<version>\n`

Sent exactly once by the Pi immediately after the serial connection is
opened (before any `S` command), so both sides can detect a protocol
mismatch before drive commands start flowing.

```
V,<version>\n
```

- `<version>`: a non-negative integer, the protocol version number. This
  document defines **version `1`**.
- Example: `V,1\n`
- The receiving MCU firmware SHOULD compare this against the version(s) it
  supports and refuse to arm the drive motors (or otherwise fail safe) if it
  does not recognize the version. Negotiation/fallback across versions is
  out of scope for v1 — a mismatch is a hard incompatibility.
- The Pi-side client is not required to wait for an acknowledgment from the
  MCU after sending `V,1\n` in v1 (no ack message is defined). This is a
  known v1 limitation (see Section 7).

### 2.2 Drive command — `S,<speed>,<steer>\n`

The steady-state message, sent repeatedly (per decision-loop tick) by the
Pi to command the car's speed tier and steering offset.

```
S,<speed>,<steer>\n
```

Exactly two fields after the `S` tag, in this order: `speed` then `steer`.
No other fields are permitted in a v1 `S` message.

---

## 3. Field definitions

### 3.1 `speed` — speed tier (integer enum)

`speed` is an ASCII-encoded integer, one of exactly three values:

| Value | Name  | Meaning                                   |
|-------|-------|--------------------------------------------|
| `0`   | STOP  | Motors stopped / zero throttle             |
| `1`   | SLOW  | Reduced-throttle tier (obstacle nearby)    |
| `2`   | FULL  | Full/normal-throttle tier (path clear)     |

- No other integer values are valid. No decimals, no leading `+`, no
  leading zeros (e.g. `01` is invalid), no whitespace.
- `0` (STOP) is the safety-critical value — see Section 6 (Watchdog).

### 3.2 `steer` — steering offset (signed integer)

`steer` is an ASCII-encoded signed integer in the closed range:

```
-100 ..= 100
```

- **Unit**: percentage of maximum steering deflection (i.e. `100` means
  "full deflection to one side," `0` means "straight ahead," `-100` means
  "full deflection to the other side"). It is not degrees and not raw servo
  microseconds — mapping this percentage to actual servo PWM/angle is a
  firmware-side concern outside this spec.
- **Sign convention**: **negative = steer left, positive = steer right**
  (from the car's forward-facing point of view).
- `0` is the neutral/straight value and MUST always be valid.
- Encoding rules: optional leading `-` for negative values only (never a
  leading `+` for positive values); no decimals; no leading zeros beyond a
  bare `0`; no whitespace. Valid examples: `0`, `5`, `-5`, `100`, `-100`.
  Invalid examples: `+5`, `05`, `5.0`, ` 5`, `101`, `-101`.

---

## 4. Framing and delimiter rules

- **Field separator**: `,` (single comma, no surrounding whitespace).
- **Line terminator**: `\n` (single LF byte). Every message, with no
  exception, ends in exactly one `\n`.
- **No embedded delimiters in fields**: since all v1 fields are integers,
  fields can never legitimately contain `,` or `\n`. Any occurrence of
  these inside what would otherwise be a field is by definition a framing
  violation, not a data value.
- **Fixed field count per message type**: `V` messages have exactly 1 field
  after the tag; `S` messages have exactly 2 fields after the tag. A
  message with more or fewer fields than its type requires is malformed.
- **Reading**: receivers MUST read the stream line-buffered — i.e. accumulate
  bytes until a `\n` is seen, then treat everything up to (not including)
  that `\n` as one complete message to parse. Do not attempt to parse a
  partial line before its terminating `\n` has arrived.
- **Tag character**: the first field (`V` or `S`) identifies the message
  type. Any other leading tag is unrecognized and must be treated as
  malformed (see Section 5).

---

## 5. Malformed / partial message handling

The following are all malformed input under this spec:

1. **Wrong field count** — e.g. `S,2\n` (missing `steer`), or
   `S,2,-15,0\n` (extra field).
2. **Non-numeric field** — e.g. `S,abc,5\n` (speed is not an integer),
   `S,2,left\n` (steer is not an integer).
3. **Out-of-range value** — e.g. `S,3,0\n` (speed not in {0,1,2}),
   `S,2,150\n` (steer outside -100..100).
4. **Unrecognized tag** — e.g. `X,1,2\n`.
5. **Missing terminator / truncated read** — a line that never receives its
   `\n` (e.g. the connection drops or stalls mid-message). A receiver
   reading line-buffered will simply have an incomplete buffer; it must not
   treat a partial, unterminated buffer as a complete message.
6. **Malformed numeric encoding** — leading `+`, leading zeros, decimals,
   or embedded whitespace in an integer field (e.g. `S,+2,05\n`).

**Contract**: this is a spec-level contract, not merely a suggestion. A
conforming decoder (T22's `protocol.py`) MUST reject every case above by
raising a specific `ProtocolError` (or equivalent well-defined exception in
whatever language the MCU firmware is written in) rather than:

- silently substituting a default value,
- silently truncating/clamping an out-of-range value into range, or
- returning a partially-populated/garbage result.

Receivers must fail loudly on anything that doesn't exactly match this
spec. There is no "best effort" parse mode in v1.

---

## 6. Watchdog / STOP semantics

`S,0,0\n` (STOP, steer neutral) is the safety-critical message in this
protocol:

- It MUST always be a syntactically valid, always-parseable message per
  this spec — it is not a special case exempted from framing/encoding
  rules, it is simply the most important instance of an ordinary `S`
  message.
- The Pi-side watchdog (outside the scope of this doc, see
  `src/rccar/watchdog/`) sends `S,0,0\n` whenever it detects a failure
  condition (stale frame, processing stall, serial write failure, etc.),
  as its forced fail-safe action.
- Because the MCU firmware cannot rely on any single message arriving
  (serial link can drop bytes), firmware implementers are strongly
  encouraged to implement their own independent timeout: if no valid `S`
  message (of any kind) has been received within a firmware-defined
  timeout window, the firmware should stop the motors on its own,
  independent of receiving an explicit `S,0,0\n`. This local failsafe
  requirement is a firmware design note, not a wire-format requirement,
  and is called out here only because it directly motivates why STOP must
  always be trivially parseable.

---

## 7. Scope note — what v1 explicitly does NOT cover

The following are known, intentional gaps in v1 and are deferred to a
future protocol version:

- **Resync after garbled bytes.** If the byte stream desyncs mid-message
  (e.g. a dropped/corrupted byte causes what looks like a field with an
  embedded stray character, or a line that never gets its `\n`), v1 does
  not define any resynchronization algorithm (such as scanning forward for
  the next `\n` and discarding the partial buffer). The v1 contract is:
  the decoder raises `ProtocolError` and stops there — it is the caller's
  responsibility to decide how to recover the stream (e.g. discard buffer
  and wait for the next `\n`). This is flagged as a post-v1 gap requiring
  real robustness work, not something a v1 decoder does automatically.
- **No acknowledgment/handshake protocol** beyond the one-shot `V,<n>\n`
  header — no per-message ack, no retry/resend logic.
- **No checksum/CRC** — a bit-flip in a numeric field that still happens to
  produce a syntactically valid, in-range message will not be detected.
- **No version negotiation** — a version mismatch is a hard failure
  condition for the receiver to handle (e.g. refuse to arm), not something
  the protocol auto-negotiates around.

Full serial-link robustness (partial reads, garbled-byte resync) is
explicitly out of scope for v1 per the project plan; decoders must fail
loudly (raise `ProtocolError`), never silently return wrong values.

---

## 8. Worked examples

### 8.1 Valid messages

| Bytes on the wire | Meaning |
|---|---|
| `V,1\n` | Version header, protocol version 1. Sent once at connect. |
| `S,2,-15\n` | Speed tier FULL (2), steer 15% deflection to the **left**. |
| `S,1,10\n` | Speed tier SLOW (1), steer 10% deflection to the **right**. |
| `S,0,0\n` | STOP, steering neutral. The watchdog fail-safe message. |

### 8.2 Malformed messages (MUST raise `ProtocolError` on decode)

| Bytes on the wire | Why it's malformed |
|---|---|
| `S,2\n` | Missing the `steer` field — `S` requires exactly 2 fields. |
| `S,abc,5\n` | `speed` field (`abc`) is not an integer. |
| `S,3,0\n` | `speed` value `3` is not one of {0, 1, 2}. |
| `S,2,150\n` | `steer` value `150` is outside the -100..100 range. |

---

## 9. Summary reference

```
V,<version>\n                  version, sent once on connect
S,<speed>,<steer>\n            drive command, sent every decision tick

speed  ::= "0" | "1" | "2"     0=STOP  1=SLOW  2=FULL
steer  ::= integer in [-100, 100]   % of max deflection, negative=left, positive=right
```
