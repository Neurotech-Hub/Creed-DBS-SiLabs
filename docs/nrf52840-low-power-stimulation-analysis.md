# nRF52840 Low-Power Stimulation Analysis

## Scope

This analysis compares the current MouseStim v8.1 implementation against an nRF52840-based MCU/software upgrade. It uses `docs/mousestim_current_architecture.md` as the reference for measured current and hardware architecture.

The intended comparison is intentionally constrained:

- Main proposed upgrade: MCU and stimulation-control software.
- Hardware outside the MCU is assumed mostly unchanged unless explicitly called out.
- BLE is assumed minimal and is not the primary optimization target.
- The goal is to estimate potential functional gain in battery life and chronic stimulation current by changing how pulse timing is generated.

Typical stimulation range evaluated:

- Frequency: `8-160 Hz`
- Pulse duration: `50-600 us`

## Executive Recommendation

An nRF52840 MCU/software upgrade is most compelling if the current `1.47 mA` measured "sleep" state is caused primarily by the BGM220S remaining in an active or semi-active timing mode. In that case, replacing CPU-timed stimulation with RTC + hardware-timed pulse windows could plausibly save hundreds of microamps to more than 1 mA at the battery level.

The upgrade is less compelling if the measured sleep and stimulation currents are dominated by the fixed power architecture: always-on +5 V boost, 3.3 V LDO from +5 V, -5 V charge pump, DAC/reference, op amp, analog switch, and Howland network DC paths. Those costs do not disappear from an MCU swap.

Recommended nRF52840 control architecture:

- RTC for inter-pulse cadence and future burst envelopes.
- Short TIMER/PWM window only around pulse phases.
- PPI/GPIOTE for deterministic electrode GPIO edges.
- SPIM EasyDMA for DAC frame updates.
- CPU wake only for configuration, train start/end, error handling, and infrequent re-arming.

Bottom line: nRF52840 is a plausible power-function upgrade, but the equitable target is not "microamp total device current." The fair target is reducing the MCU/timing portion of a presently `1.47-3.42 mA` measured system while recognizing that the analog stimulation chain can still set a floor near the mA scale.

## Current Implementation Baseline

Measured system current from the current MouseStim architecture:

| Mode | Measured avg current | Lifetime to 90% battery | Interpretation |
|---|---:|---:|---|
| Sleep | `1.47 mA` | 14.29 days | Likely not true low-power MCU sleep or includes always-on fixed rail overhead. |
| BLE advertising | `1.62 mA` | 12.96 days | `+0.15 mA` over measured sleep. |
| BLE connected | `1.89 mA` | 11.11 days | `+0.42 mA` over measured sleep. |
| Stimulating, 60 uA | `3.15 mA` | 6.67 days | `+1.26 mA` over BLE connected. |
| Stimulating, 400 uA | `3.42 mA` | 6.14 days | Only `+0.27 mA` over 60 uA stimulation. |

The important observation is that therapeutic amplitude is not the dominant current term. Going from 60 uA to 400 uA stimulation adds only `0.27 mA`, while the fixed overhead from sleep to connected stimulation is much larger.

## Current Hardware Constraints

The current architecture matters because an MCU swap alone does not remove all fixed loads:

- Battery feeds MAX17220 boost to +5 V.
- The 3.3 V MCU rail is generated from +5 V through TPS709.
- The negative rail uses MAX1853 from +5 V.
- Stimulation uses LT6656 reference, AD5683 DAC, LT6020 dual op amp / Howland pump, and ADG1236 electrode switch.

Battery-equivalent multipliers from the architecture note:

- +5 V load: `I_batt ~= 1.96 * I_5V`
- 3.3 V load through +5 V boost/LDO path: `I_batt ~= 1.29 * I_3V3`
- +/-5 V op-amp load across 10 V span: `I_batt ~= 3.92 * I_supply`

This makes a direct MCU comparison stricter: even if nRF52840 enters microamp System ON sleep, any remaining always-on +5 V, -5 V, DAC, op-amp, or resistor-network current still dominates the system budget.

## Requirements Inferred From Current Firmware

From existing `app.c` behavior:

- Periodic stimulation cadence is frequency-driven.
- Pulse timing is microsecond-scale with empirical offsets.
- Two-phase output is used: stimulation then charge-balance.
- Charge-balance duration is approximately `5x` pulse duration.
- DAC updates are performed through 3-byte SPI frames.
- Output gating uses `SHDN`, `ELEC0_OUT`, and `ELEC1_OUT`.
- The current code uses blocking SPI and a busy-wait op-amp settle delay during stimulation startup.

Implication: a lower-power MCU implementation needs both low-power long-interval scheduling and precise short-window edge timing. The direct functional improvement is replacing CPU wake/busy-wait/ISR pulse work with hardware-timed pulse sequencing.

## nRF52840 Capability Fit

Relevant nRF52840 datapoints:

- System ON wake on event, no RAM retention: about `0.97 uA`
- System ON wake on RTC, no RAM retention: about `1.5 uA`
- System ON wake on RTC, full RAM retention: about `3.16 uA`
- One 1 MHz TIMER running from HFINT: about `418 uA`
- CPU running CoreMark at 64 MHz from flash with DC/DC: about `3.3 mA`

Design interpretation:

- RTC-driven inter-pulse sleep can be extremely low at the MCU level.
- A continuously running high-frequency TIMER is not low power.
- The nRF52840 gain comes from duty-cycling high-frequency resources, not from leaving them running.
- PPI/GPIOTE and SPIM EasyDMA are the functional advantages: they can reduce CPU participation in pulse edges and DAC transfers.

## Equitable MCU/Software Power Model

The model below estimates only the nRF52840 pulse-timing portion. It excludes unchanged analog loads, power conversion losses from those loads, therapeutic electrode current, and BLE.

Assumptions:

- Op-amp/set-up guard window: `120 us`
- Charge-balance ratio: `5`
- Active pulse window:

`active_us = 120 + pulse_us * (1 + 5)`

- Duty:

`duty = active_us * frequency_hz / 1,000,000`

- MCU timing average:

`I_mcu_timing ~= Isleep * (1 - duty) + Itimer * duty + Icpu_rearm`

Where:

- `Isleep = 1.5 uA`
- `Itimer = 418 uA`
- Example CPU re-arm term: `50 us` at `3.3 mA` per pulse

### nRF52840 Timing-Only Scenario Results

| Scenario | Frequency | Pulse | Active window | Duty | RTC+TIMER only | + 50 us CPU re-arm |
|---|---:|---:|---:|---:|---:|---:|
| Lowest typical | 8 Hz | 50 us | 420 us | 0.34% | 2.9 uA | 4.2 uA |
| Moderate | 40 Hz | 200 us | 1320 us | 5.28% | 23.5 uA | 30.1 uA |
| DBS-like | 130 Hz | 200 us | 1320 us | 17.16% | 73.0 uA | 94.4 uA |
| Highest typical | 160 Hz | 600 us | 3720 us | 59.52% | 249.5 uA | 275.9 uA |

Converted through the current +5 V -> 3.3 V supply path, these MCU timing numbers are roughly `1.29x` at the battery:

| Scenario | nRF52840 timing current at MCU rail | Battery-equivalent through current rail path |
|---|---:|---:|
| Lowest typical | 4.2 uA | 5.4 uA |
| Moderate | 30.1 uA | 38.8 uA |
| DBS-like | 94.4 uA | 121.8 uA |
| Highest typical | 275.9 uA | 355.9 uA |

These are attractive relative to the measured mA-scale system current, but only for the MCU timing portion.

## Comparison Against Current Measured System

The current measured stimulation current is:

- `3.15 mA` at 60 uA stimulation
- `3.42 mA` at 400 uA stimulation

An MCU/software-only nRF52840 change should be evaluated as a reduction of the current MCU/timing overhead, not as removal of the analog overhead.

Plausible fixed analog and rail contributors from the architecture note:

| Contributor | Expected battery current | MCU/software leverage |
|---|---:|---|
| BGM220S active CPU/timing without deep sleep | `~0.7-1.9 mA` | High; this is the main replacement target. |
| BLE connected average overhead | `~0.3-0.5 mA` | Medium; scoped as minimal but connection policy still matters. |
| LT6020 dual op amp on +/-5 V | `~0.7-0.8 mA` | Limited if chronic stimulation keeps it active. |
| MAX1853 negative rail quiescent | `~0.3-0.6 mA` | Limited unless negative rail can be disabled between trains. |
| AD5683 DAC active | `~0.22-0.35 mA` external ref, up to `~0.7-1.0 mA` internal ref | Some; avoid internal ref, avoid per-pulse rewrites, power down between trains if allowed. |
| LT6656 + DAC reference input | `~0.05-0.1 mA` | Low-medium. |
| ADG1236 | typical near zero, worst-case could be `~0.1-0.5 mA` | Medium; ensure valid logic states. |
| Howland resistor network DC paths | unknown, possibly `~0.1-1+ mA` | Mostly hardware/common-mode dependent. |
| Therapeutic current increase, 60 uA -> 400 uA | measured `+0.27 mA` | Medium; not the dominant fixed cost. |

### Practical Gain Estimate

If the current BGM220S timing/control contribution is near `0.7-1.9 mA` battery equivalent, and nRF52840 hardware-timed stimulation is around `0.006-0.356 mA` battery equivalent across the typical stimulation range, then the MCU/software upgrade could plausibly save:

- Low/mid stimulation settings: roughly `0.6-1.8 mA` if the current MCU is active most of the time.
- High duty setting (`160 Hz`, `600 us`): roughly `0.3-1.5 mA`, because the nRF52840 timing window itself rises to about `0.356 mA` battery equivalent.

This maps to a possible total stimulation current improvement from `3.15-3.42 mA` down toward roughly `1.6-2.8 mA`, depending on how much of the current sleep/stim overhead is truly MCU-driven and how much is analog/rail-driven.

That range is intentionally conservative. It does not assume removal of the +5 V boost, -5 V charge pump, op amp, DAC, or Howland static currents.

## Architecture Options Comparison

| Option | Power expectation | Timing quality | Fit to current hardware | Recommendation |
|---|---|---|---|---|
| RTC + short TIMER/PPI/GPIOTE/SPIM window | Best MCU/software-only balance | Good to very good | Fits current DAC/GPIO style | **Preferred baseline** |
| RTC + PWM sequence | Potentially good | Very good for edges | Less direct if DAC must change phase-by-phase | Evaluate if GPIO timing dominates |
| Pure CPU ISR sequencing | Better than busy-wait if sleep is used, but leaves savings unused | Variable under BLE/interrupt load | Easy to port | Prototype only |
| Continuous HF TIMER | Poor, about `418 uA` just for 1 MHz timer | Good | Technically easy | Avoid for chronic operation |
| External latch/analog switch changes | Potentially best | High | Outside MCU/software scope | Future hardware spin only |

## Functional Gain From nRF52840

The main functional gain is not only lower sleep current. It is a better way to express stimulation timing:

- RTC can represent long inter-pulse gaps and future burst envelopes cleanly.
- TIMER/PWM can represent microsecond pulse phases without CPU busy-waiting.
- PPI/GPIOTE can move electrode GPIO edge timing out of interrupt latency.
- SPIM EasyDMA can reduce CPU time spent moving DAC bytes.
- CPU can be reserved for parameter changes, train start/end, safety checks, BLE events, and fault handling.

This directly addresses the current chronic-stimulation use case: amplitude, pulse frequency, pulse duration, and likely future burst patterns.

## Risks And Assumptions

- If the measured `1.47 mA` sleep current is mostly fixed power-rail or analog leakage, nRF52840 will not recover that full amount.
- If +5 V must remain on solely because the MCU VDD rail is derived from it, the current architecture limits how low total sleep can go.
- If the analog chain must remain continuously powered during chronic stimulation, the op amp, -5 V inverter, DAC, and Howland network set a mA-scale floor.
- DAC phase updates may still require careful orchestration; EasyDMA reduces CPU byte movement but does not remove analog settling.
- Timing must be validated with BLE present, even if BLE duty is low.

## Minimum Validation Before Hardware Commitment

1. Measure current rail decomposition in the current design:
   - battery input to MAX17220,
   - +5 V rail,
   - 3.3 V VDD rail,
   - MAX1853 input,
   - LT6020 supply,
   - AD5683/LT6656 supply.
2. Measure current and timing at:
   - `8 Hz / 50 us`,
   - representative mid-setting,
   - `130 Hz / 200 us`,
   - `160 Hz / 600 us`.
3. Capture logic traces for `SHDN`, `ELEC0_OUT`, `ELEC1_OUT`, SPI `CS/SCLK`, and DAC settling.
4. Confirm whether AD5683 internal reference is disabled when LT6656 is used.
5. Confirm LED, debugger, RTT/SWO/logging, and floating GPIOs are not contributing to measured sleep.
6. Prototype nRF52840 timing on a dev kit and measure MCU rail current for RTC sleep, TIMER window, PPI/GPIOTE edges, and SPIM EasyDMA DAC-like transfers.

## Go/No-Go Statement

**Go (moderate confidence)** if measurements show the current MCU/timing/control budget is a major contributor to the `1.47-3.42 mA` measured system current and nRF52840 can keep HF resources duty-cycled across required stimulation settings.

**Conditional go** if the nRF52840 saves only hundreds of microamps but also improves timing architecture, burst support, and firmware simplicity enough to justify the migration.

**No-go or defer** if rail decomposition shows the current sleep/stim currents are dominated by always-on +5 V/-5 V/analog loads that remain unchanged by the MCU and software upgrade.
