# MouseStim v8.1 Current-Consumption Architecture Notes

Purpose: architecture/current budget handoff for an MCU/software-focused agent optimizing chronic implantable DBS operation.

Source hardware: `MouseStim_v8.1.kicad_sch` / `MouseStim_v8.1.pdf`, BOM supplied in prompt.

## 1. Measured system current

Battery basis: two coin cells in parallel, treated as ~3.0 V nominal and 560 mAh total. User-provided lifetime assumes 90% usable capacity = 504 mAh.

| Mode | Measured avg current | Lifetime to 90% battery | Notes |
|---|---:|---:|---|
| Sleep | 1.47 mA | 14.29 days | Likely not true BGM220S EM2/EM3 sleep. |
| BLE Advertising | 1.62 mA | 12.96 days | +0.15 mA over measured sleep. |
| BLE Connected | 1.89 mA | 11.11 days | +0.42 mA over measured sleep. |
| Stimulating, 60 µA | 3.15 mA | 6.67 days | +1.26 mA over BLE connected. |
| Stimulating, 400 µA | 3.42 mA | 6.14 days | +0.27 mA over 60 µA stimulation. |

Main interpretation: stimulation amplitude is not the dominant cost. The dominant active-mode cost is fixed overhead: MCU timing/radio state, +5 V generation, -5 V generation, DAC/reference/op-amp/switching, and possibly DC current in the Howland network.

## 2. Power architecture

```text
2x coin cells in parallel (~3 V, 560 mAh total)
        |
        v
U1 MAX17220 boost converter
        |
        +---- +5 V rail -----------------------------------------------+
        |                                                              |
        |                                                              v
        |                                                      U7 MAX1853
        |                                                       -5 V rail
        |
        +---- U10 TPS709 LDO -> VDD / 3.3 V rail
                            |
                            +--> IC2 BGM220S MCU/BLE module
                            +--> U3 DRV5032 Hall sensor
                            +--> LED0 path
                            +--> digital logic/control pins

Stim analog chain:
U5 LT6656 1.25 V reference -> U4 AD5683 DAC -> U6 LT6020-1 dual op amp / Howland pump
                                                     |
                                                     v
                                            U9 ADG1236 electrode switch
                                                     |
                                                     v
                                             electrode/filter network
```

## 3. Estimate assumptions

These are not final measurements. They are first-pass expected currents for prioritizing firmware and MCU work.

| Assumption | Value |
|---|---:|
| Coin cell bus voltage | 3.0 V nominal |
| +5 V boost efficiency for estimates | 85% |
| Battery current for +5 V load | `I_batt ≈ I_5V × 5 / (3 × 0.85) ≈ 1.96 × I_5V` |
| Battery current for 3.3 V LDO load, via +5 V boost | `I_batt ≈ I_3V3 × 3.3 / (3 × 0.85) ≈ 1.29 × I_3V3` |
| Battery current for ±5 V op-amp load | `I_batt ≈ I_supply × 10 / (3 × 0.85) ≈ 3.92 × I_supply`, where `I_supply` is the op-amp supply current flowing across both rails. |
| MAX17220 quiescent | Sub-µA class; ignored in mA-scale active budget. |
| MAX1853 charge-pump efficiency | Not separately modeled; quiescent and negative-rail loads should be measured at +5 V input if possible. |

## 4. Component-level expected current budget

### 4.1 MCU / radio / always-on digital

| Ref | Part/value | Rail | Role | Expected component current | Battery-equivalent estimate | MCU/software leverage | Priority |
|---|---|---:|---|---:|---:|---|---|
| IC2 | BGM220SC12WGA2R | 3.3 V VDD | BLE + pulse timing/control | EM0 active: ~22-38 µA/MHz. At 38.4 MHz, ~0.84-1.46 mA before radio. EM1: ~13-17 µA/MHz, ~0.50-0.65 mA at 38.4 MHz. EM2: ~1.4 µA. RX active ~4.2 mA during packet activity. TX 0 dBm ~4.6 mA, 6 dBm ~8.8 mA during TX. | EM0 @38.4 MHz: ~1.1-1.9 mA battery. EM1 @38.4 MHz: ~0.65-0.84 mA battery. Radio bursts: ~5.4-11.4 mA battery while active. | Highest software target. Avoid busy loops. Use TIMER/PRS/PWM for pulse edges. Use LDMA/SPI for DAC updates. Drop HCLK when possible. Enter EM1 between pulse edges. Duty-cycle BLE. Avoid debugger/RTT/logging during current tests. | High |
| U3 | DRV5032-OMNIPOLAR | 3.3 V VDD | Magnet/wake/control input | Version-dependent: ~0.54 µA at 5 Hz, ~1.3 µA at 20 Hz, ~5.2 µA at 80 Hz. | <0.01 mA | Minimal. Confirm exact suffix/sample rate. Use lowest-rate option if compatible. | Low |
| D1/R1 | LED + 1 kΩ | 3.3 V VDD | Status indicator | Off: ~0. On: roughly `(3.3 V - Vf) / 1 kΩ`, commonly ~1-2 mA. | On: ~1.3-2.6 mA battery | Ensure LED is disabled during chronic operation. Avoid heartbeat blink unless explicitly needed. | High if currently active |
| Y1 | Crystal | BGM220S oscillator domain | LF timing reference | Included in BGM220S mode current. LFRCO/LFXO currents are sub-µA class. | Included above | Use LF timer for coarse scheduling; avoid HF oscillator when not required. | Medium |
| AE1 | Chip antenna | RF | BLE antenna | No DC current. | 0 | BLE TX power/interval tuning only. | Low |
| U2/J1/TPs | Programming/debug/test | VDD/GPIO | SWD/debug/header/test points | No intended current unless debugger attached or pins are driven/leaking. | 0 nominal | Measure with debugger disconnected. Disable SWO/RTT/logging. Avoid floating inputs. | Medium |

### 4.2 Power conversion

| Ref | Part/value | Rail | Role | Expected component current | Battery-equivalent estimate | MCU/software leverage | Priority |
|---|---|---:|---|---:|---:|---|---|
| U1 | MAX17220ELT_T | battery -> +5 V | Always-on boost converter | Sub-µA quiescent class, but all +5 V and 3.3 V loads pass through this converter. | Load multiplier: +5 V loads cost ~1.96× at battery. 3.3 V loads cost ~1.29× at battery. | None if it must stay on for MCU VDD. Major architectural issue: MCU VDD depends on +5 V. | High architecture |
| U10 | TPS709 3.3 V LDO | +5 V -> 3.3 V | BGM220S/VDD regulator | ~1 µA IQ; ~150 nA shutdown. LDO itself is low-Iq. | IQ negligible, but every 3.3 V load pays boost + LDO path. LDO power loss is `(5 V - 3.3 V) × I_3V3`. | Little software leverage because it powers MCU. | Medium architecture |
| U7 | MAX1853 | +5 V -> -5 V | Negative rail for analog stim | ~165 µA typical / 320 µA max quiescent at 25 °C, shutdown near nA/0.1 µA class. | Quiescent alone: ~0.32 mA battery typical, ~0.63 mA max. Additional -5 V loads increase this. | Disable when not stimulating. During chronic stimulation it is fixed overhead unless pulses can be grouped/duty-cycled. | High |
| L2/C8/C9/C3/C4/C5/C6/C12/C16/C17/C18/C19 | Inductors/caps | Power rails | Energy storage/decoupling | No DC current ideally; leakage usually negligible relative to mA budget. | ~0 | Firmware cannot improve. Component sizing/ESR affects converter efficiency/noise. | Low |
| D2 | Schottky | -5 V clamp/protection | Protection around negative rail | No intended DC current except leakage or clamp events. | Usually negligible | None. Verify it is not forward-biased during stimulation. | Low |

### 4.3 Analog stimulation chain

| Ref | Part/value | Rail | Role | Expected component current | Battery-equivalent estimate | MCU/software leverage | Priority |
|---|---|---:|---|---:|---:|---|---|
| U6 | LT6020IDD-1-PBF | +5 V / -5 V | Dual op amp, Howland pump | ~90 µA/amplifier typical, 100 µA/amplifier max. Dual: ~180-200 µA across a 10 V supply span. Shutdown ~1.4 µA typical. | Active quiescent: ~0.70-0.78 mA battery. Shutdown: ~0.006 mA battery. | If chronic continuous stimulation, little unless stimulation is duty-cycled. If there are inter-train gaps, assert shutdown. Avoid leaving enabled outside active pulse windows. | High |
| U4 | AD5683 | likely +5 V / VDDA | DAC amplitude command | Normal mode, external reference/internal ref disabled: ~110 µA typ / 180 µA max. Internal reference enabled: ~350 µA typ / 500 µA max. Power-down: ~2 µA. | External-ref normal: ~0.22-0.35 mA battery. Internal-ref normal: ~0.69-0.98 mA battery. Power-down: ~0.004 mA battery. | Disable internal reference if using LT6656. Update DAC only when amplitude changes. Consider power-down between trains if wake time acceptable. Use DMA/SPI rather than CPU polling. | High |
| U5 | LT6656-1.25 | +5 V | Precision 1.25 V DAC/reference | <1 µA supply current; can source load current into DAC VREF. AD5683 reference input current may be ~26 µA at gain=1 or ~47 µA at gain=2. | LT6656 alone <0.002 mA battery. With DAC VREF load: ~0.05-0.09 mA battery. | Keep powered only when DAC/stim active if feasible. Ensure DAC internal reference is off. | Low-Medium |
| U9 | ADG1236 | +5 V / -5 V, 3.3 V logic inputs | Electrode switch / polarity routing | Typical power dissipation <0.03 µW. Datasheet supports 3 V logic-compatible inputs. Distributor max supply current may be hundreds of µA worst-case at high supply/temp; likely much lower at ±5 V. | Typical negligible; worst-case could be ~0.1-0.5 mA battery-equivalent. | Ensure IN1/IN2 never float. Drive only valid logic levels. Reduce unnecessary toggling. Disconnect electrodes before rail/power sequencing. | Medium |
| R2/R3/R5/R6/R11/R12/R13/R14/R15 | 10 kΩ / 12 kΩ / 2 kΩ Howland resistors | analog | Howland set/feedback network | Unknown without node voltages. Static DC could be non-trivial, especially through 2 kΩ elements if they sit across multi-volt differences. | Could plausibly be ~0.1-1+ mA battery depending waveform/common-mode. Must measure or SPICE. | Software can reduce DC bias/common-mode where possible. Hardware change: scale network up 5-10× while preserving ratios if bandwidth/noise/offset allow. | High measurement |
| R7/R8/R16/R17 + C10/C11/C14/C15 | 330 Ω + 470 pF | electrode filters | Output EMI/filtering | Capacitors: no DC. Resistors: only electrode/stimulation current and transient pulse current. | Included in stimulation load. | Pulse shape/frequency/amplitude affect energy. | Medium |
| Electrodes/load | mouse tissue/electrode interface | output | Actual therapeutic current | Measured delta from 60 µA to 400 µA stimulation is +0.27 mA battery. | Empirical: +0.27 mA battery for +340 µA programmed current. | Optimize compliance voltage and pulse width/frequency. Avoid excess headroom/common-mode. | Medium |

### 4.4 Passive / connector components

| Ref | Part/value | Role | Expected current | Notes |
|---|---|---|---:|---|
| C1/C3-C19 | capacitors | Decoupling, charge pump, output filtering | ~0 DC | Leakage generally negligible relative to mA budget; value/ESR can affect converter efficiency and analog stability. |
| L2 | 2.2 µH | MAX17220 boost inductor | Not a load | DCR affects efficiency. |
| R4/R14/R15 etc. | 10 kΩ pulls/biases | Logic/analog bias | Depends on placement | Any 10 kΩ from rail to ground costs 0.33 mA at 3.3 V or 0.5 mA at 5 V. Confirm none are simple always-on rail dividers. |
| J1 | 4-pin connector | Electrode/output connector | 0 | Current only through load. |
| TP2/TP3/TP7/TP8/TP10/TP11 | test points | Rail/output debug | 0 | Useful for adding sense resistors/cut-jumpers in next spin. |
| BT1/BT2 | coin cells | Energy source | n/a | 560 mAh total assumed from user measurements. |

## 5. Measurement reconciliation: where 3.42 mA likely goes

This is a plausible allocation, not a measured decomposition.

| Contributor | Expected battery current | Confidence | Notes |
|---|---:|---|---|
| BGM220S active CPU/timing without deep sleep | ~0.7-1.9 mA | Medium | Depends heavily on HCLK, EM0 vs EM1, stack state, and busy-wait timing. |
| BLE connected average overhead | ~0.3-0.5 mA | High | User measured +0.42 mA sleep -> connected. |
| LT6020 dual op amp on ±5 V | ~0.7-0.8 mA | High | Fixed chronic stim overhead. |
| MAX1853 -5 V inverter quiescent | ~0.3-0.6 mA | High | Fixed while negative rail enabled. |
| AD5683 DAC active | ~0.22-0.35 mA if external ref; ~0.7-1.0 mA if internal ref left on | High | Confirm control register disables internal reference. |
| LT6656 + DAC VREF input | ~0.05-0.1 mA | Medium | Reference itself is tiny; DAC VREF input current may dominate. |
| ADG1236 | ~0 to 0.5 mA | Low-Medium | Usually tiny; verify at ±5 V and actual logic states. |
| Howland resistor network DC paths | unknown; possibly ~0.1-1+ mA | Low | Needs rail-current measurement or SPICE with actual waveform/node voltages. |
| Electrode current increase from 60 µA to 400 µA | +0.27 mA measured | High | Actual therapeutic current is not dominant. |
| LED if accidentally active | +1.3-2.6 mA | High if on | Easy to rule out. |

Expected takeaway: the 3.42 mA active current is plausible if the MCU is active, the ±5 V stim analog chain is continuously powered, the DAC is active, and there is some additional DC current in the Howland network.

## 6. Software/MCU optimization checklist

### 6.1 Pulse generation

Goal: do not bit-bang pulse edges from a CPU loop.

Preferred approach:

1. Precompute stimulation parameters:
   - phase width
   - inter-phase delay
   - pulse period
   - train duration
   - amplitude DAC code
   - switch polarity sequence
2. Use hardware timers for phase edges.
3. Use PRS/PWM/GPIO routing where possible for deterministic output transitions.
4. Use LDMA/SPI for DAC writes if amplitude changes within a train.
5. Wake CPU only at:
   - train start
   - train end
   - error/fault
   - parameter update
   - BLE event
6. Keep CPU in EM1 between hardware events rather than EM0 busy-waiting.

Expected benefit:
- Moving from EM0 busy-wait at 38.4 MHz to EM1 at 38.4 MHz may save roughly `(0.84-1.46 mA) - (0.50-0.65 mA)` at the BGM220S, equivalent to about `0.25-1.05 mA` battery current depending on clock and activity.
- Lowering HCLK further during chronic stimulation can save proportionally if peripherals still meet timing.

### 6.2 BLE behavior

For chronic stimulation, BLE should be a control/telemetry channel, not necessarily a continuously connected high-duty radio link.

Actions:
- Increase connection interval.
- Use slave latency if acceptable.
- Reduce TX power if link margin allows.
- Disconnect outside reprogramming/telemetry windows.
- Advertise slowly when not actively being configured.
- Batch telemetry rather than streaming continuously.
- Avoid debug logging over RTT/SWO/UART during current tests.

### 6.3 DAC/reference control

Actions:
- Confirm AD5683 internal reference is disabled when LT6656 is used.
- Program amplitude once per parameter change, not every pulse, unless waveform requires it.
- Use hardware LDAC timing if needed.
- Power down DAC between trains if stimulation is not truly continuous.
- If power-down is used with reference disabled, note wake time may be much longer than with output buffer only.

### 6.4 Rail and GPIO sequencing

Safe low-power or inter-train sequence:

1. Finish current pulse phase.
2. Open/disconnect electrode switch to safe state.
3. Set DAC output to safe baseline.
4. Put DAC in power-down if allowed.
5. Disable LT6020 if not needed.
6. Disable MAX1853 if negative rail not needed.
7. Leave +5 V active only if MCU VDD architecture requires it.
8. Set GPIOs connected to unpowered analog chips to high-Z or safe levels to avoid back-powering.

Wake/stim sequence:

1. Enable +5 V if not already active.
2. Enable -5 V charge pump.
3. Wait for rail settling.
4. Enable/reference/DAC.
5. Program DAC amplitude.
6. Enable LT6020.
7. Wait for analog settling.
8. Connect electrodes/switches.
9. Start timer-driven pulse train.

### 6.5 Debug/test conditions

When measuring current:
- Disconnect debugger.
- Disable RTT/SWO/logging.
- Compile a release build.
- Confirm no GPIO is floating.
- Force LED off.
- Measure rail currents separately using jumpers or temporary sense resistors.

## 7. Recommended next measurements

Add temporary 10 Ω sense resistors or cut-jumpers and measure these rails during chronic stimulation:

| Measurement | Why |
|---|---|
| Battery input to U1 MAX17220 | Total system reference. |
| +5 V rail output | Separates boost/load effects from battery behavior. |
| 3.3 V VDD rail | Quantifies BGM220S + Hall + LED + logic. |
| U7 MAX1853 input current | Quantifies negative rail fixed overhead. |
| U6 LT6020 supply current | Confirms op-amp quiescent + output load. |
| U4/U5 DAC/reference supply | Confirms DAC internal reference state and VREF current. |
| U9 ADG1236 supply | Rules out logic-state supply current issue. |
| Howland resistor network branch currents | Identifies whether 2 kΩ/10 kΩ/12 kΩ values are burning meaningful DC. |

## 8. Design-change ideas outside MCU/software scope

These are useful context for the software agent but are hardware revisions.

1. Separate always-on MCU rail from +5 V stim rail.
   - Current design powers MCU through +5 V boost -> 3.3 V LDO.
   - A dedicated low-Iq 3.0/3.3 V rail would let +5 V become stim-only.
2. Scale Howland resistor network upward.
   - Try 10× values: 2 kΩ -> 20 kΩ, 10 kΩ -> 100 kΩ, 12 kΩ -> 120 kΩ.
   - Preserve ratios; verify noise, settling, offset, and leakage sensitivity.
3. Replace continuous ±5 V architecture if not required.
   - If stimulation can be single-supply with active recharge/electrode shorting, remove -5 V overhead.
4. Consider a lower-power current-source architecture.
   - The LT6020 is efficient for a precision op amp, but ±5 V continuous analog overhead is still large relative to a coin-cell implant.

## 9. Source notes

Primary schematic/BOM basis:
- `MouseStim_v8.1.pdf`, uploaded by user.

Datasheet values used:
- Silicon Labs BGM220S datasheet: https://www.silabs.com/documents/public/data-sheets/bgm220s-datasheet.pdf
- Analog Devices MAX17220/MAX17225 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/max17220-max17225.pdf
- TI TPS709 datasheet: https://www.ti.com/lit/ds/symlink/tps709.pdf
- TI DRV5032 datasheet: https://www.ti.com/lit/gpn/DRV5032
- Analog Devices LT6020/LT6020-1 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/60201fa.pdf
- Analog Devices LT6656 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/lt6656.pdf
- Analog Devices AD5683R/AD5682R/AD5681R/AD5683 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ad5683r_5682r_5681r_5683.pdf
- Analog Devices MAX1852/MAX1853 datasheet: https://www.mouser.com/datasheet/2/609/MAX1852_MAX1853-3128038.pdf
- Analog Devices ADG1236 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adg1236.pdf
