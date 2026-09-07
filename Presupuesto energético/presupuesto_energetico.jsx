import { useState, useMemo } from "react";
import { BarChart, Bar, XAxis, YAxis, Tooltip, Cell, ResponsiveContainer, CartesianGrid } from "recharts";

// ─────────────────────────────────────────────────────────────
// CONSTANTES — todas las corrientes en mA
// ─────────────────────────────────────────────────────────────
const MCU = {
  xiao_c3: { label: "XIAO ESP32-C3",     iA: 51.0, iS: 0.0451, hasLP: false, chargeMa: 370,  charger: "ETA4054 — 370 mA integrado" },
  xiao_c6: { label: "XIAO ESP32-C6",     iA: 51.0, iS: 0.0143, hasLP: true,  chargeMa: null, charger: "Por confirmar (esquemático pendiente)" },
  supermini:{ label: "ESP32-C3 SuperMini",iA: 51.0, iS: 0.0430, hasLP: false, chargeMa: 1000, charger: "TP4056 ext. — 1000 mA" },
};

const EDA = {
  grove:   { label: "Grove GSR",     iA: 1.500, iI: 0.000, note: "Power-gated en IDLE" },
  mcp6002: { label: "MCP6002 TIA",   iA: 0.090, iI: 0.090, note: "Consumo constante" },
};

// Sensores fijos — iA = activo (SAMPLE), iI = idle (IDLE)
const SENSORS = [
  { id: "ad8232",  label: "AD8232 (ECG)",   iA: 0.170, iI: 0.170, always: true },
  { id: "ads1115", label: "ADS1115 (ADC)",  iA: 0.200, iI: 0.002 },
  { id: "mpu6050", label: "MPU-6050 (IMU)", iA: 3.600, iI: 0.140 },
  { id: "max30205",label: "MAX30205 (Temp)",iA: 0.600, iI: 0.001 },
];

// MicroSD: ráfaga 30 mA × 50 ms por ciclo + power-down en IDLE
const SD_BURST_MAS = 30 * 0.050; // 1.5 mA·s
const SD_IDLE_MA   = 0.050;

// Colores de la interfaz
const C = {
  bg: "#080d1a", card: "#0f172a", border: "#1e293b",
  sample: "#60a5fa", idle: "#34d399", avg: "#c084fc",
  ok: "#22c55e", warn: "#f59e0b", err: "#f87171",
  text: "#f1f5f9", muted: "#64748b", hl: "#1a2d4f",
  ad8232: "#fbbf24", mpu: "#fb923c", eda: "#e879f9",
  sd: "#a78bfa", other: "#34d399",
};
const mono = { fontFamily: "'JetBrains Mono','Fira Mono',monospace" };

// ─────────────────────────────────────────────────────────────
// UTILIDADES
// ─────────────────────────────────────────────────────────────
const fmt = (mA) => {
  if (mA >= 1)     return mA.toFixed(2) + " mA";
  if (mA >= 0.001) return (mA * 1000).toFixed(1) + " µA";
  return (mA * 1000).toFixed(3) + " µA";
};

function compute(mcuKey, edaKey, tS, tI, batMah, batEff, alertPct) {
  const m = MCU[mcuKey], e = EDA[edaKey];
  const tC = tS + tI;
  const dc = tS / tC;

  const rows = [
    { id: "mcu",    label: "MCU (HP core)",      iA: m.iA,             iI: m.iS },
    ...SENSORS,
    { id: "microsd",label: "MicroSD",             iA: SD_BURST_MAS / tS, iI: SD_IDLE_MA },
    { id: "eda",    label: `EDA — ${e.label}`,   iA: e.iA,             iI: e.iI },
  ];

  const iSamp = rows.reduce((s, r) => s + r.iA, 0);
  const iIdle = rows.reduce((s, r) => s + r.iI, 0);
  const iNorm = iSamp * dc + iIdle * (1 - dc);
  const iInt  = m.iA + 0.170 + 0.200 + 3.600 + e.iA + 0.600 + 5.0;
  const iAvg  = iNorm * (1 - alertPct / 100) + iInt * (alertPct / 100);
  const usable = batMah * batEff / 100;
  const hours  = usable / iAvg;
  const chargeH = m.chargeMa ? (batMah * 1.1) / m.chargeMa : null;

  return { rows, iSamp, iIdle, iNorm, iInt, iAvg, usable, hours, dc, tC, chargeH };
}

const SHORT_LABEL = { mcu: "MCU", ad8232: "AD8232", ads1115: "ADS1115", mpu6050: "IMU", max30205: "Temp", microsd: "SD", eda: "EDA" };
const CHIP_COLOR  = { mcu: C.sample, ad8232: C.ad8232, ads1115: C.idle, mpu6050: C.mpu, max30205: C.idle, microsd: C.sd, eda: C.eda };

const PRESETS = [
  { lbl: "Baseline (3s / 5s)",         tS: 3, tI: 5,  req: null },
  { lbl: "Optimizado (3s / 8s)",        tS: 3, tI: 8,  req: null },
  { lbl: "LP — defensible (3s / 17s)", tS: 3, tI: 17, req: "LP Core + validación clínica" },
  { lbl: "LP — conservador (3s / 27s)",tS: 3, tI: 27, req: "LP Core + aprobación neuróloga" },
  { lbl: "LP — agresivo (3s / 57s)",   tS: 3, tI: 57, req: "LP Core + aprobación neuróloga" },
];

// ─────────────────────────────────────────────────────────────
// COMPONENTE PRINCIPAL
// ─────────────────────────────────────────────────────────────
export default function EnergyBudget() {
  const [mcuKey, setMcu]    = useState("xiao_c6");
  const [edaKey, setEda]    = useState("grove");
  const [lpCore, setLp]     = useState(false);
  const [batMah, setBat]    = useState(450);
  const [batEff, setEff]    = useState(85);
  const [tSamp,  setTS]     = useState(3);
  const [tIdle,  setTI]     = useState(8);
  const [alertP, setAlert]  = useState(1);

  const mcu  = MCU[mcuKey];
  const lpOn = lpCore && mcu.hasLP;

  const r = useMemo(
    () => compute(mcuKey, edaKey, tSamp, tIdle, batMah, batEff, alertP),
    [mcuKey, edaKey, tSamp, tIdle, batMah, batEff, alertP]
  );

  // Escenarios: agrega configuración actual si no coincide con un preset
  const scenarios = useMemo(() => {
    const list = PRESETS.map(s => ({ ...s, ...compute(mcuKey, edaKey, s.tS, s.tI, batMah, batEff, alertP), curr: false }));
    const isPreset = PRESETS.some(s => s.tS === tSamp && s.tI === tIdle);
    if (!isPreset) list.push({ lbl: "▶ Config. actual", tS: tSamp, tI: tIdle, req: null, curr: true, ...compute(mcuKey, edaKey, tSamp, tIdle, batMah, batEff, alertP) });
    return list.map(s => ({ ...s, curr: s.curr || (s.tS === tSamp && s.tI === tIdle) }));
  }, [mcuKey, edaKey, tSamp, tIdle, batMah, batEff, alertP]);

  const chartData = r.rows
    .map(row => ({ id: row.id, name: SHORT_LABEL[row.id] || row.id, avg: +(row.iA * r.dc + row.iI * (1 - r.dc)).toFixed(4) }))
    .sort((a, b) => b.avg - a.avg);

  const autColor = r.hours >= 28 ? C.ok : r.hours >= 20 ? C.warn : C.err;
  const sWarn = tSamp < 3;
  const iWarn = !lpOn && tIdle > 10;

  // ── Estilos locales ──
  const card = { background: C.card, borderRadius: "10px", padding: "14px", border: `1px solid ${C.border}` };
  const sectionHead = { fontSize: "10px", fontWeight: 700, color: C.muted, textTransform: "uppercase", letterSpacing: "0.06em", marginBottom: "10px" };

  const Slider = ({ label, value, min, max, step, onChange, color = C.sample, unit = "" }) => (
    <div style={{ marginBottom: "12px" }}>
      <div style={{ display: "flex", justifyContent: "space-between", marginBottom: "2px" }}>
        <span style={{ fontSize: "11px", color: C.muted }}>{label}</span>
        <span style={{ ...mono, fontSize: "12px", fontWeight: 700, color }}>{value}{unit}</span>
      </div>
      <input type="range" min={min} max={max} step={step} value={value}
        onChange={e => onChange(+e.target.value)}
        style={{ width: "100%", accentColor: color, cursor: "pointer", height: "4px" }} />
    </div>
  );

  const Badge = ({ color, children }) => (
    <span style={{ fontSize: "10px", padding: "2px 6px", borderRadius: "100px", background: color + "22", color, border: `1px solid ${color}44`, fontWeight: 600 }}>{children}</span>
  );

  return (
    <div style={{ background: C.bg, color: C.text, minHeight: "100vh", padding: "14px", fontFamily: "'Inter',system-ui,sans-serif", fontSize: "13px" }}>

      {/* ── Encabezado ── */}
      <div style={{ marginBottom: "12px", display: "flex", alignItems: "baseline", gap: "12px", flexWrap: "wrap" }}>
        <div>
          <div style={{ fontSize: "17px", fontWeight: 800, letterSpacing: "-0.02em" }}>⚡ Presupuesto Energético Interactivo</div>
          <div style={{ color: C.muted, fontSize: "11px", marginTop: "1px" }}>Wearable Epilepsia Refractaria · mA · Datasheets + PPK2</div>
        </div>
        <Badge color={autColor}>{r.hours.toFixed(1)} h autonomía</Badge>
        <Badge color={C.avg}>{(r.dc * 100).toFixed(0)}% duty cycle HP</Badge>
      </div>

      {/* ── Fila principal 3 columnas ── */}
      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1.05fr", gap: "10px", marginBottom: "10px" }}>

        {/* Col 1: Componentes */}
        <div style={card}>
          <div style={sectionHead}>🔧 Componentes</div>

          <div style={{ fontSize: "11px", color: C.muted, marginBottom: "4px" }}>Microcontrolador</div>
          <select value={mcuKey}
            onChange={e => { setMcu(e.target.value); if (!MCU[e.target.value].hasLP) setLp(false); }}
            style={{ width: "100%", padding: "6px 8px", borderRadius: "6px", background: C.bg, border: `1px solid ${C.border}`, color: C.text, fontSize: "12px", marginBottom: "8px" }}>
            {Object.entries(MCU).map(([k, v]) => <option key={k} value={k}>{v.label}</option>)}
          </select>

          {mcu.hasLP && (
            <label style={{ display: "flex", gap: "8px", alignItems: "flex-start", background: "#052e16", border: "1px solid #15803d", borderRadius: "6px", padding: "8px", marginBottom: "8px", cursor: "pointer" }}>
              <input type="checkbox" checked={lpCore} onChange={e => setLp(e.target.checked)} style={{ marginTop: "2px", accentColor: C.ok }} />
              <div>
                <div style={{ fontWeight: 700, color: "#4ade80", fontSize: "11px" }}>LP Core activo</div>
                <div style={{ color: "#86efac", fontSize: "10px", marginTop: "1px" }}>Monitoreo IMU en deep sleep · MPU-6050 en LP I²C GPIO6/7</div>
              </div>
            </label>
          )}
          {!mcu.hasLP && <div style={{ height: "8px" }} />}

          <div style={{ fontSize: "11px", color: C.muted, marginBottom: "4px" }}>Sensor EDA</div>
          <div style={{ display: "flex", gap: "5px", marginBottom: "10px" }}>
            {Object.entries(EDA).map(([k, v]) => (
              <button key={k} onClick={() => setEda(k)} style={{
                flex: 1, padding: "6px 4px", borderRadius: "6px", border: "none", cursor: "pointer", fontSize: "11px", fontWeight: 600,
                background: edaKey === k ? C.sample : C.border, color: edaKey === k ? "#0a0f1e" : C.muted, transition: "all 0.15s"
              }}>{v.label}</button>
            ))}
          </div>

          <div style={{ background: C.bg, borderRadius: "6px", padding: "10px", fontSize: "11px" }}>
            {[
              ["Sleep MCU",    `${(mcu.iS * 1000).toFixed(1)} µA`],
              ["Cargador",     mcu.charger],
              ["EDA — IDLE",   EDA[edaKey].note],
              ["EDA — activo", fmt(EDA[edaKey].iA)],
            ].map(([k, v]) => (
              <div key={k} style={{ display: "flex", justifyContent: "space-between", marginBottom: "5px", gap: "8px" }}>
                <span style={{ color: C.muted, flexShrink: 0 }}>{k}</span>
                <span style={{ ...mono, color: C.text, textAlign: "right", wordBreak: "break-all", fontSize: "10px" }}>{v}</span>
              </div>
            ))}
          </div>
        </div>

        {/* Col 2: Ciclo + Batería */}
        <div style={card}>
          <div style={sectionHead}>⏱ Ciclo & Batería</div>

          <Slider label="Ventana SAMPLE — HP activo" value={tSamp} min={1} max={10} step={0.5} onChange={setTS} color={C.sample} unit=" s" />
          <div style={{ fontSize: "10px", marginTop: "-8px", marginBottom: "10px", color: sWarn ? C.warn : C.ok }}>
            {sWarn
              ? "⚠ Mínimo 3 s para HRV (necesita 3 picos R a 60 lpm)"
              : `✓ ~${Math.max(0, Math.round(tSamp * 1.25) - 1)} R-R @ 75 lpm · ${Math.round(tSamp * 250)} muestras ADS1115`}
          </div>

          <Slider label={`IDLE — HP en sleep${lpOn ? " (LP Core activo)" : ""}`} value={tIdle} min={3} max={lpOn ? 120 : 30} step={1} onChange={setTI} color={C.idle} unit=" s" />
          <div style={{ fontSize: "10px", marginTop: "-8px", marginBottom: "10px", color: iWarn ? C.warn : lpOn && tIdle > 10 ? C.idle : C.muted }}>
            {iWarn
              ? `⚠ IMU sin monitorear ${tIdle} s — habilitar LP Core`
              : lpOn && tIdle > 10
                ? "✓ LP Core mantiene monitoreo IMU continuo en sleep"
                : `${(r.dc * 100).toFixed(0)}% HP activo · ${(3600 / r.tC).toFixed(0)} ciclos/hora`}
          </div>

          <Slider label="% tiempo en modo INTENSIVO" value={alertP} min={0} max={10} step={0.1} onChange={setAlert} color={C.warn} unit="%" />
          <div style={{ fontSize: "10px", marginTop: "-8px", marginBottom: "12px", color: C.muted }}>
            ≈ {(alertP / 100 * 24 * 60).toFixed(0)} min/día · {r.iInt.toFixed(0)} mA constante en INTENSIVO
          </div>

          <div style={{ borderTop: `1px solid ${C.border}`, paddingTop: "10px" }}>
            <div style={{ fontSize: "10px", color: C.muted, marginBottom: "6px" }}>🔋 Batería</div>
            <Slider label="Capacidad" value={batMah} min={200} max={1000} step={50} onChange={setBat} color={C.warn} unit=" mAh" />
            <Slider label="Eficiencia estimada" value={batEff} min={70} max={95} step={1} onChange={setEff} color={C.warn} unit="%" />
            <div style={{ display: "flex", justifyContent: "space-between", fontSize: "11px" }}>
              <span style={{ color: C.muted }}>Capacidad usable</span>
              <span style={{ ...mono, fontWeight: 700, color: C.warn }}>{r.usable.toFixed(0)} mAh</span>
            </div>
          </div>
        </div>

        {/* Col 3: Resultado */}
        <div style={{ ...card, border: `1.5px solid ${autColor}50`, display: "flex", flexDirection: "column", alignItems: "center", textAlign: "center" }}>
          <div style={{ fontSize: "10px", fontWeight: 700, color: autColor, textTransform: "uppercase", letterSpacing: "0.08em", marginBottom: "4px" }}>
            {r.hours >= 24 ? "✓ Objetivo alcanzado" : r.hours >= 20 ? "⚠ Objetivo en riesgo" : "✗ Objetivo no alcanzado"}
          </div>

          <div style={{ fontSize: "72px", fontWeight: 900, color: autColor, lineHeight: 1, ...mono }}>
            {r.hours.toFixed(1)}
          </div>
          <div style={{ fontSize: "13px", color: C.muted, marginBottom: "12px" }}>horas de autonomía</div>

          {/* Barra de progreso */}
          <div style={{ width: "100%", background: C.bg, borderRadius: "100px", height: "8px", position: "relative", marginBottom: "18px" }}>
            <div style={{ width: `${Math.min(r.hours / 48 * 100, 100)}%`, height: "8px", background: autColor, borderRadius: "100px", transition: "width 0.35s ease" }} />
            <div style={{ position: "absolute", left: "50%", top: "-6px", width: "1.5px", height: "20px", background: C.muted }} />
            <div style={{ position: "absolute", left: "50%", bottom: "-16px", transform: "translateX(-50%)", fontSize: "10px", color: C.muted }}>24 h</div>
          </div>

          <div style={{ fontSize: "12px", color: r.hours >= 24 ? C.ok : C.err, marginBottom: "12px" }}>
            {r.hours >= 24
              ? `+${(r.hours - 24).toFixed(1)} h de margen (+${((r.hours / 24 - 1) * 100).toFixed(0)}%)`
              : `Déficit: ${(24 - r.hours).toFixed(1)} h para el objetivo`}
          </div>

          <div style={{ background: C.bg, borderRadius: "8px", padding: "10px", width: "100%", fontSize: "11px" }}>
            {[
              ["SAMPLE",            `${r.iSamp.toFixed(1)} mA`,  C.sample],
              ["IDLE",              fmt(r.iIdle),                 C.idle],
              ["Promedio normal",   `${r.iNorm.toFixed(2)} mA`,  C.avg],
              ["Prom. c/ alerta",   `${r.iAvg.toFixed(2)} mA`,   C.text],
              ["Duty cycle HP",     `${(r.dc * 100).toFixed(0)}%`, C.muted],
            ].map(([k, v, col]) => (
              <div key={k} style={{ display: "flex", justifyContent: "space-between", marginBottom: "5px" }}>
                <span style={{ color: C.muted }}>{k}</span>
                <strong style={{ color: col, ...mono }}>{v}</strong>
              </div>
            ))}
            {r.chargeH != null && (
              <div style={{ borderTop: `1px solid ${C.border}`, paddingTop: "6px", marginTop: "4px", display: "flex", justifyContent: "space-between" }}>
                <span style={{ color: C.muted }}>Carga completa</span>
                <strong style={{ color: r.chargeH > 12 ? C.warn : C.ok, ...mono }}>~{r.chargeH.toFixed(1)} h</strong>
              </div>
            )}
            {r.chargeH == null && (
              <div style={{ borderTop: `1px solid ${C.border}`, paddingTop: "6px", marginTop: "4px", fontSize: "10px", color: C.warn }}>
                ⚠ Corriente de carga pendiente confirmar
              </div>
            )}
          </div>
        </div>
      </div>

      {/* ── Gráfico + Tabla ── */}
      <div style={{ display: "grid", gridTemplateColumns: "1.15fr 1fr", gap: "10px", marginBottom: "10px" }}>

        {/* Gráfico horizontal */}
        <div style={card}>
          <div style={sectionHead}>📊 Contribución promedio por componente — duty cycle {(r.dc * 100).toFixed(0)}%</div>
          <ResponsiveContainer width="100%" height={240}>
            <BarChart data={chartData} layout="vertical" margin={{ top: 0, right: 50, left: 44, bottom: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke={C.border} horizontal={false} />
              <XAxis type="number" tick={{ fill: C.muted, fontSize: 10 }}
                tickFormatter={v => v >= 1 ? v.toFixed(0) + " mA" : (v * 1000).toFixed(0) + " µA"} />
              <YAxis type="category" dataKey="name" tick={{ fill: C.text, fontSize: 11 }} width={42} />
              <Tooltip
                formatter={v => [fmt(v), "Aporte promedio"]}
                contentStyle={{ background: C.card, border: `1px solid ${C.border}`, borderRadius: "6px", fontSize: "11px" }}
                labelStyle={{ color: C.text, fontWeight: 700 }}
              />
              <Bar dataKey="avg" radius={[0, 4, 4, 0]}>
                {chartData.map(e => <Cell key={e.id} fill={CHIP_COLOR[e.id] || C.idle} />)}
              </Bar>
            </BarChart>
          </ResponsiveContainer>
        </div>

        {/* Tabla de desglose */}
        <div style={card}>
          <div style={sectionHead}>📋 Desglose SAMPLE / IDLE / Promedio</div>
          <div style={{ overflowX: "auto" }}>
            <table style={{ width: "100%", borderCollapse: "collapse", fontSize: "11px" }}>
              <thead>
                <tr style={{ borderBottom: `1px solid ${C.border}` }}>
                  {["Componente", "SAMPLE", "IDLE", "Prom."].map((h, i) => (
                    <th key={h} style={{ padding: "4px 6px", textAlign: i === 0 ? "left" : "right", color: [C.muted, C.sample, C.idle, C.avg][i], fontWeight: 700 }}>{h}</th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {r.rows.map(row => {
                  const avg = row.iA * r.dc + row.iI * (1 - r.dc);
                  return (
                    <tr key={row.id} style={{ borderBottom: `1px solid ${C.border}33` }}>
                      <td style={{ padding: "4px 6px", color: row.always ? C.ad8232 : C.text }}>{row.label}</td>
                      <td style={{ padding: "4px 6px", textAlign: "right", color: C.sample, ...mono }}>{fmt(row.iA)}</td>
                      <td style={{ padding: "4px 6px", textAlign: "right", color: C.idle,   ...mono }}>{fmt(row.iI)}</td>
                      <td style={{ padding: "4px 6px", textAlign: "right", color: C.avg,    ...mono, fontWeight: 700 }}>{fmt(avg)}</td>
                    </tr>
                  );
                })}
              </tbody>
              <tfoot>
                <tr style={{ borderTop: `1px solid ${C.border}` }}>
                  <td style={{ padding: "5px 6px", fontWeight: 700, color: C.text }}>TOTAL</td>
                  <td style={{ padding: "5px 6px", textAlign: "right", color: C.sample, ...mono, fontWeight: 700 }}>{r.iSamp.toFixed(2)} mA</td>
                  <td style={{ padding: "5px 6px", textAlign: "right", color: C.idle,   ...mono, fontWeight: 700 }}>{fmt(r.iIdle)}</td>
                  <td style={{ padding: "5px 6px", textAlign: "right", color: C.avg,    ...mono, fontWeight: 700 }}>{r.iNorm.toFixed(2)} mA</td>
                </tr>
              </tfoot>
            </table>
          </div>
          <div style={{ marginTop: "8px", fontSize: "10px", color: C.muted, borderTop: `1px solid ${C.border}`, paddingTop: "6px", lineHeight: 1.5 }}>
            <span style={{ color: C.ad8232 }}>■</span> AD8232 sin modo sleep — 170 µA constante en SAMPLE e IDLE.<br />
            MicroSD: burst 30 mA × 50 ms/ciclo asignado a SAMPLE; 50 µA power-down en IDLE.<br />
            MPU-6050 IDLE: modo LP accel 40 Hz — 140 µA (necesario para monitoreo motor).
          </div>
        </div>
      </div>

      {/* ── Tabla de escenarios ── */}
      <div style={card}>
        <div style={sectionHead}>
          🔍 Escenarios de Referencia — {mcu.label} · {EDA[edaKey].label} · {batMah} mAh · {batEff}% efic. · {alertP.toFixed(1)}% alerta
        </div>
        <div style={{ overflowX: "auto" }}>
          <table style={{ width: "100%", borderCollapse: "collapse", fontSize: "11px" }}>
            <thead>
              <tr style={{ borderBottom: `1px solid ${C.border}`, color: C.muted, fontSize: "10px" }}>
                {["Escenario", "tS", "tI", "Ciclo", "Duty", "I SAMPLE", "I IDLE", "I Prom.", "Autonomía", "Requisitos"].map((h, i) => (
                  <th key={h} style={{ padding: "5px 7px", textAlign: i < 2 ? "left" : "right", fontWeight: 600, whiteSpace: "nowrap" }}>{h}</th>
                ))}
              </tr>
            </thead>
            <tbody>
              {scenarios.map((s, i) => {
                const ac = s.hours >= 28 ? C.ok : s.hours >= 20 ? C.warn : C.err;
                return (
                  <tr key={i} style={{ borderBottom: `1px solid ${C.border}33`, background: s.curr ? C.hl : "transparent" }}>
                    <td style={{ padding: "5px 7px", color: s.curr ? "#93c5fd" : C.text, fontWeight: s.curr ? 700 : 400, whiteSpace: "nowrap" }}>{s.lbl}</td>
                    <td style={{ padding: "5px 7px", textAlign: "right", ...mono }}>{s.tS} s</td>
                    <td style={{ padding: "5px 7px", textAlign: "right", ...mono }}>{s.tI} s</td>
                    <td style={{ padding: "5px 7px", textAlign: "right", ...mono }}>{s.tC} s</td>
                    <td style={{ padding: "5px 7px", textAlign: "right", ...mono, color: C.muted }}>{(s.dc * 100).toFixed(0)}%</td>
                    <td style={{ padding: "5px 7px", textAlign: "right", ...mono, color: C.sample }}>{s.iSamp.toFixed(1)} mA</td>
                    <td style={{ padding: "5px 7px", textAlign: "right", ...mono, color: C.idle }}>{fmt(s.iIdle)}</td>
                    <td style={{ padding: "5px 7px", textAlign: "right", ...mono, color: C.avg }}>{s.iNorm.toFixed(2)} mA</td>
                    <td style={{ padding: "5px 7px", textAlign: "right", ...mono, fontWeight: 800, color: ac, whiteSpace: "nowrap" }}>
                      {s.hours.toFixed(1)} h {s.hours >= 24 ? "✓" : "✗"}
                    </td>
                    <td style={{ padding: "5px 7px", textAlign: "right", fontSize: "10px", color: s.req ? C.warn : C.border, whiteSpace: "nowrap" }}>
                      {s.req || "—"}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
        <div style={{ marginTop: "8px", fontSize: "10px", color: C.muted, lineHeight: 1.5 }}>
          Escenarios LP Core requieren XIAO ESP32-C6 con LP Core habilitado y MPU-6050 en bus LP I²C independiente (GPIO6/7 del chip).<br />
          IDLE &gt; 10 s requiere validación clínica del intervalo de muestreo HRV con el equipo de neurología.
        </div>
      </div>

      <div style={{ textAlign: "center", marginTop: "10px", fontSize: "10px", color: "#1e293b" }}>
        Fuentes: datasheets oficiales · PPK2 Seeed Forum 2024 · ESP32-C3 activo sin radio ≈ 51 mA @ 160 MHz · MCP6002: 90 µA · Grove GSR: 1.5 mA
      </div>
    </div>
  );
}
