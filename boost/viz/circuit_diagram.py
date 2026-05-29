def generate_boost_circuit_svg(params, results):
    Vin = params["Vin"]
    R = params["R"]
    L_val = params["L_uH"]
    C_val = params["C_uF"]
    f_val = params["f_kHz"]
    RL_val = params["RL"]
    Vd_val = params["Vd"]
    
    # 确保类型安全
    D_val = float(results.get("D", params.get("D", 0.5)))
    Vout_val = float(results.get("Vout", 0))
    mode_val = str(results.get("mode", "CCM"))

    svg = f'''
    <svg viewBox="0 0 520 280" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;width:100%;">
      <style>
        .wire {{ stroke:#333; stroke-width:2; fill:none; }}
        .label {{ font-family:Consolas,monospace; font-size:11px; fill:#1a73e8; }}
        .comp {{ font-family:Consolas,monospace; font-size:12px; fill:#333; font-weight:bold; }}
        .value {{ font-family:Consolas,monospace; font-size:10px; fill:#e8710a; }}
        .gnd {{ stroke:#333; stroke-width:2; fill:none; }}
      </style>

      <circle cx="40" cy="80" r="18" class="wire"/>
      <line x1="32" y1="74" x2="48" y2="74" class="wire"/>
      <line x1="40" y1="74" x2="40" y2="68" class="wire"/>
      <line x1="36" y1="86" x2="44" y2="86" class="wire"/>
      <line x1="40" y1="86" x2="40" y2="98" class="wire"/>
      <text x="20" y="52" class="comp">Vin</text>
      <text x="20" y="112" class="value">{Vin}V</text>
      <line x1="40" y1="62" x2="40" y2="80" class="wire"/>
      <line x1="58" y1="80" x2="90" y1="80" class="wire"/>

      <path d="M90,80 C95,65 105,65 110,80 C115,65 125,65 130,80 C135,65 145,65 150,80" class="wire"/>
      <text x="110" y="56" class="comp">L</text>
      <text x="90" y="112" class="value">{L_val}μH (RL={RL_val}Ω)</text>

      <line x1="150" y1="80" x2="190" y2="80" class="wire"/>
      <text x="175" y="68" class="label" font-size="9">node A</text>

      <line x1="190" y1="80" x2="190" y2="110" class="wire"/>
      <rect x="178" y="110" width="24" height="40" rx="3" style="stroke:#333;stroke-width:2;fill:#f0f4ff"/>
      <text x="184" y="133" class="comp" font-size="10">SW</text>
      <line x1="190" y1="150" x2="190" y2="200" class="wire"/>
      <text x="210" y="126" class="value">D={D_val:.2f}</text>
      <text x="210" y="140" class="value">f={f_val}kHz</text>

      <line x1="190" y1="80" x2="210" y2="80" class="wire"/>
      <polygon points="210,74 210,86 226,80" style="stroke:#333;stroke-width:2;fill:#fff8e1"/>
      <line x1="226" y1="74" x2="226" y2="86" class="wire"/>
      <line x1="226" y1="80" x2="300" y2="80" class="wire"/>
      <text x="212" y="56" class="comp">D</text>
      <text x="212" y="112" class="value">Vd={Vd_val}V</text>

      <line x1="300" y1="80" x2="400" y2="80" class="wire"/>

      <line x1="300" y1="80" x2="300" y2="120" class="wire"/>
      <line x1="288" y1="120" x2="312" y2="120" class="wire"/>
      <line x1="288" y1="128" x2="312" y2="128" class="wire"/>
      <line x1="300" y1="128" x2="300" y2="200" class="wire"/>
      <text x="318" y="126" class="comp">C</text>
      <text x="318" y="140" class="value">{C_val}μF</text>

      <line x1="400" y1="80" x2="400" y2="110" class="wire"/>
      <rect x="388" y="110" width="24" height="40" rx="2" style="stroke:#333;stroke-width:2;fill:#ffe0b2"/>
      <text x="396" y="133" class="comp">R</text>
      <text x="420" y="133" class="value">{R}Ω</text>
      <line x1="400" y1="150" x2="400" y2="200" class="wire"/>

      <line x1="400" y1="80" x2="450" y2="80" class="wire"/>
      <circle cx="450" cy="80" r="3" style="fill:#333;"/>
      <text x="440" y="64" class="comp">Vout</text>
      <text x="440" y="100" class="value" style="fill:#1a73e8;font-weight:bold;font-size:11px;">{Vout_val:.2f}V</text>
      <text x="440" y="115" class="value" style="fill:#1a73e8;font-size:10px;">({mode_val})</text>

      <line x1="40" y1="200" x2="450" y2="200" class="wire"/>

      <line x1="40" y1="200" x2="40" y2="210" class="wire"/>
      <line x1="35" y1="210" x2="45" y2="210" class="gnd"/>
      <line x1="37" y1="215" x2="43" y2="215" class="gnd"/>
      <line x1="39" y1="220" x2="41" y2="220" class="gnd"/>

      <line x1="190" y1="200" x2="190" y2="210" class="wire"/>
      <line x1="185" y1="210" x2="195" y2="210" class="gnd"/>
      <line x1="187" y1="215" x2="193" y2="215" class="gnd"/>
      <line x1="189" y1="220" x2="191" y2="220" class="gnd"/>

      <line x1="300" y1="200" x2="300" y2="210" class="wire"/>
      <line x1="295" y1="210" x2="305" y2="210" class="gnd"/>
      <line x1="297" y1="215" x2="303" y2="215" class="gnd"/>
      <line x1="299" y1="220" x2="301" y2="220" class="gnd"/>

      <line x1="400" y1="200" x2="400" y2="210" class="wire"/>
      <line x1="395" y1="210" x2="405" y2="210" class="gnd"/>
      <line x1="397" y1="215" x2="403" y2="215" class="gnd"/>
      <line x1="399" y1="220" x2="401" y2="220" class="gnd"/>

      <text x="65" y="42" class="label" font-size="10">→ IL</text>
    </svg>
    '''
    return svg