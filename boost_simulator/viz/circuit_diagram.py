def generate_boost_circuit_svg(params, results):
    Vin = params["Vin"]
    R = params["R"]
    L_val = params["L_uH"]
    C_val = params["C_uF"]
    f_val = params["f_kHz"]
    RL_val = params["RL"]
    Vd_val = params["Vd"]
    D_val = results.get("D", params.get("D", 0.5))
    Vout_val = results.get("Vout", 0)
    mode_val = results.get("mode", "CCM")

    svg = f'''<svg viewBox="0 0 800 360" xmlns="http://www.w3.org/2000/svg" style="width:100%;max-width:800px;">
      <defs>
        <style>
          .w {{ stroke:#2c3e50; stroke-width:2.5; fill:none; stroke-linecap:round; }}
          .cb {{ font-family:'Segoe UI',Arial,sans-serif; font-size:14px; fill:#2c3e50; font-weight:600; }}
          .vl {{ font-family:'Segoe UI',Arial,sans-serif; font-size:12px; fill:#e67e22; }}
          .io {{ font-family:'Segoe UI',Arial,sans-serif; font-size:13px; fill:#2980b9; font-weight:600; }}
          .nd {{ font-family:'Segoe UI',Arial,sans-serif; font-size:10px; fill:#7f8c8d; }}
          .bx {{ stroke:#2c3e50; stroke-width:2; }}
        </style>
      </defs>

      <circle cx="50" cy="80" r="5" style="fill:#e74c3c"/>
      <text x="30" y="68" class="io">Vin+</text>
      <line x1="55" y1="80" x2="120" y1="80" class="w"/>

      <path d="M120,80 C130,55 145,55 155,80 C165,55 180,55 190,80 C200,55 215,55 225,80 C235,55 250,55 260,80" class="w"/>
      <text x="175" y="46" class="cb">L</text>
      <text x="140" y="108" class="vl">{L_val}μH</text>
      <text x="140" y="122" class="vl">RL={RL_val}Ω</text>

      <line x1="260" y1="80" x2="340" y1="80" class="w"/>
      <circle cx="340" cy="80" r="4" style="fill:#2c3e50"/>
      <text x="330" y="68" class="nd">SW node</text>

      <line x1="340" y1="80" x2="340" y2="110" class="w"/>
      <rect x="320" y="110" width="40" height="50" rx="4" class="bx" style="fill:#ebf5fb"/>
      <text x="332" y="140" class="cb" font-size="11">MOSFET</text>
      <line x1="340" y1="160" x2="340" y2="260" class="w"/>
      <text x="365" y="125" class="vl">D={D_val:.2f}</text>
      <text x="365" y="142" class="vl">f={f_val}kHz</text>

      <polygon points="350,72 350,88 380,80" class="bx" style="fill:#fef9e7"/>
      <line x1="380" y1="72" x2="380" y2="88" class="w" stroke-width="3"/>
      <line x1="340" y1="80" x2="350" y1="80" class="w"/>
      <line x1="380" y1="80" x2="520" y1="80" class="w"/>
      <text x="355" y="46" class="cb">Diode</text>
      <text x="345" y="108" class="vl">Vd={Vd_val}V</text>

      <circle cx="520" cy="80" r="4" style="fill:#2c3e50"/>

      <line x1="520" y1="80" x2="520" y2="140" class="w"/>
      <line x1="505" y1="140" x2="535" y2="140" class="w" stroke-width="3"/>
      <line x1="505" y1="155" x2="535" y2="155" class="w" stroke-width="3"/>
      <line x1="520" y1="155" x2="520" y2="260" class="w"/>
      <text x="540" y="148" class="cb">C</text>
      <text x="540" y="165" class="vl">{C_val}μF</text>

      <line x1="520" y1="80" x2="570" y1="80" class="w"/>
      <path d="M570,80 L578,68 L594,92 L610,68 L626,92 L634,80" class="w"/>
      <line x1="634" y1="80" x2="690" y1="80" class="w"/>
      <text x="585" y="46" class="cb">R</text>
      <text x="580" y="108" class="vl">{R}Ω</text>

      <circle cx="690" cy="80" r="5" style="fill:#27ae60"/>
      <text x="700" y="68" class="io">Vout+</text>
      <text x="700" y="108" class="io" style="font-size:14px">{Vout_val:.2f}V</text>
      <text x="700" y="126" class="vl" style="fill:#8e44ad;font-weight:600">{mode_val}</text>

      <line x1="50" y1="260" x2="690" y2="260" class="w"/>

      <circle cx="50" cy="260" r="5" style="fill:#e74c3c"/>
      <text x="30" y="278" class="io">Vin-</text>
      <line x1="50" y1="80" x2="50" y2="100" class="w"/>

      <circle cx="50" cy="140" r="20" class="w" style="fill:none"/>
      <line x1="42" y1="130" x2="58" y2="130" class="w"/>
      <line x1="50" y1="130" x2="50" y2="124" class="w"/>
      <line x1="42" y1="152" x2="58" y2="152" class="w"/>
      <line x1="50" y1="152" x2="50" y2="260" class="w"/>
      <text x="30" y="145" class="vl" style="fill:#e74c3c">{Vin}V</text>

      <circle cx="690" cy="260" r="5" style="fill:#27ae60"/>
      <text x="700" y="278" class="io">GND</text>
    </svg>'''

    return svg