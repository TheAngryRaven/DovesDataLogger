# RPM Spikes — Outlier Gate, RPM-Aware Noise, and a Track-Side A/B Switch

> Status: **IMPLEMENTED**, awaiting track validation. Prompted by a field
> report: a single-cylinder kart with a digital-chip magneto, whose plotted
> DOVEX `rpm` column is full of spikes that the engine plainly did not do.
>
> The reporter's hypothesis was that the filter was *over*-compensating.
> It was the opposite — see below. The `tach_filter` setting exists so that
> question gets answered with a session at the track rather than an argument
> about a graph.

## The complaint

> "I understand it's a small single cyl engine, but our magnetos do have
> digital chips in them, RPM in my opinion should appear more stable than it
> is. Are we compensating too much with the interpolation?"

## What was actually happening

There is no interpolation in the RPM path — there never was. The pipeline is:
ISR timestamps each falling edge → `TACH_LOOP()` computes the mean of the
inter-pulse periods that arrived since the last call → that mean becomes one
RPM number → a 1-D Kalman filter folds it in. The 25 Hz DOVEX row is a point
sample of whatever the filter last published.

Three defects in that chain, none of them "too much smoothing":

### 1. One bad edge went straight to the output

The estimator's steady-state gain was ~0.43 (Q = 800, R = 2500), so **43% of
every single measurement landed in the published value** — and a measurement
is one period. At 3000 RPM (a 20 ms period), an ignition ring that clears the
3 ms debounce by arriving 4 ms after a real edge reads as 15 000 RPM. The
filter had no way to tell that from the engine, so it moved ~5000 RPM. One
bad microsecond reading, one spike on the graph. A *missed* spark is the
mirror image: the period doubles, the measurement halves, the trace dives.

The 3 ms debounce is a fixed ceiling guard (~20 000 pulses/min), not a
running-speed guard. At 3000 RPM it leaves 17 ms of every period wide open to
a spurious edge.

### 2. The noise model was RPM-blind, and RPM error is quadratic in RPM

RPM = K / period, so `dRPM/dT = RPM² / K`: a **fixed** timing error is worth
**quadratically more RPM** the faster the engine turns. 80 µs of ISR latency
(the tach handler is deferred by SoftDevice radio ISRs like everything else
at app priority) costs 6 RPM at 3000 and 143 RPM at 14 000. A flat
R = 2500 RPM² (50 RPM σ) was about right at 4000 RPM and far too confident
above 8000 — so the filter trusted its noisiest measurements the most,
which is why the trace got worse the harder the engine was working.

### 3. Process noise was charged per update, not per second

Updates arrive at the *pulse* rate. A fixed Q = 800 per update means the
filter's time constant is constant in pulses and therefore **four times
looser at 12 000 RPM than at 3000** — and looser again whenever an SD GC
stall batched several pulses into one update. The amount of smoothing
silently depended on engine speed and on SD card mood.

## The fix (all in the `tach_filter` pure unit)

1. **Outlier gate.** A measurement more than `kGateSigmas` (5) from the
   estimate, measured against the combined estimate + process + measurement
   uncertainty, is not folded in — the estimate coasts. 5σ is deliberately
   loose: the gate is there for the half-period and double-period signatures
   of a spurious/missed spark, which are tens of sigma out, not to trim
   honest noise. A 5500 RPM/s pull moves the crank ~25 RPM between pulses,
   so real acceleration cannot trip it (asserted in the tests).
2. **Escape hatch.** Three *consecutive* rejections is not three coincident
   bad edges — it is a real step change (clutch dump, spin, a batch lost to
   a stall) and the estimate is the thing that is wrong. The third
   measurement is **adopted outright** rather than filtered toward, which is
   what makes an instant 11 000 → 4000 drop settle in 80 ms (faster than the
   old filter's 120 ms). Rejections must be consecutive, or a pickup emitting
   one bad edge every few seconds would eventually adopt one of them.
3. **`measurementNoise()` knows the RPM** — the quadratic timing-jitter term
   plus a linear term for genuine crank speed variation between firings
   (a single-cylinder engine really does speed up and slow down within a
   revolution; that is not measurement error, but nobody wants it plotted).
   It also knows `revsPerPulse`, so a twin's shorter periods are correctly
   treated as noisier at the same RPM.
4. **`processNoise()` is a rate** (RPM²/s) times the engine time the batch
   spans. `TACH_LOOP()` passes the sum of the periods it just consumed, not
   wall clock — at 1500 RPM those differ by 40x, and charging process noise
   for time the crank did not turn is the same mistake in a new place.
   The rate is set so the filter behaves exactly like the old one at
   6000 RPM, and unlike it, keeps behaving that way everywhere else.
5. **First measurement after a reset is adopted.** With no prior, filtering
   toward it through an RPM-aware R takes several pulses to arrive anywhere
   true — and the gate would arm partway up that climb and start rejecting
   the engine's real speed.

## Modelled result

Whole-pipeline simulation (ISR debounce → ring batching with 30 ms loop
stalls → filter → 25 Hz sampling), constant engine speed, 1% crank
variation, 150 µs ISR latency jitter, 0.5% spurious edges and 0.5% missed
sparks. Error against the true RPM, in RPM:

| Engine | legacy σ | legacy worst | **smooth σ** | **smooth worst** | raw σ |
|---|---|---|---|---|---|
| 1 500  | 501 | 6 352 | **16** | **163** | 943 |
| 3 000  | 292 | 5 657 | **19** | **286** | 446 |
| 6 000  | 420 | 5 031 | **22** |  **80** | 677 |
| 10 000 | 220 | 2 809 | **22** |  **75** | 448 |
| 14 000 | 254 | 2 977 | **22** |  **73** | 482 |

Cost, on a clean signal: a 5500 RPM/s pull lags 489 RPM instead of 267
(~90 ms). An instant 11 000 → 4000 step settles *faster*, 80 ms vs 120 ms.

Note the `raw` column. It is worse than legacy at every engine speed —
because it is the pickup, unfiltered. That is the number that answers the
original question: the filter was not manufacturing the spikes, it was
passing them through.

## The `tach_filter` setting

| Value | Behaviour |
|---|---|
| `smooth` (default) | Everything above. |
| `legacy` | The pre-0009 filter, bit for bit — fixed Q/R, no gate. Comparable against existing logs. |
| `raw` | No estimator at all: the DOVEX `rpm` column is exactly what the periods say. |

Anything unrecognised reads as `smooth`, the house idiom. The mode is read
once at boot and applied before the filter, so it affects display, DOVEX
rows, the camera FSM and the LEDs identically — there is still exactly one
RPM number in the firmware (plan 0003's rule).

**Why a setting and not just the fix.** The spikes are either the pickup or
the estimator, and no amount of reading the code settles which. `raw` for one
session shows the signal with nothing in the way; `legacy` reproduces the
logs already collected; `smooth` is what ships. Three sessions at one track
day and the question is closed.

**Deliberately not a DOVEX column.** Logging raw and filtered side by side
would A/B in a single session, but it forks the log format for a debug
feature and needs a viewer change. The setting switches what goes into the
existing `rpm` column, so nothing downstream has to know.

## Pickup-health diagnostic

With `debug_pages` = `show`, the tachometer page's subtext line becomes
`max:NNNNN S rj:NN` — the estimator in force and the number of periods the
gate has thrown away this power-cycle. It is the direct measurement of
signal quality: single digits over a session means a clean pickup; a count
that climbs with RPM is ignition ringing or missed sparks reaching the ISR.
The count deliberately survives the engine-stop reset, or it would zero at
every corner.

## Not done, on purpose

- **Adaptive (RPM-scaled) blanking in the ISR.** Modelled first, and it made
  things *worse*: blanking off the last accepted period means one spurious
  edge blanks the next real one, which manufactures a doubled period and a
  downward spike. It would have to blank off the filtered estimate, which
  puts filter state in the ISR. The gate solves the same case downstream
  with none of that.
- **Re-tuning the debounce floor.** It is a ceiling guard and correct as is
  (plan 0003). Ringing that clears it is the gate's problem now.
- **Time-window averaging of the logged column.** The estimator is now
  quiet enough (σ ≈ 20 RPM) that resampling would only add lag.
