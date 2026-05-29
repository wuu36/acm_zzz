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
        <line x1="40" y1="82" x2="40" y2="68" class="wire"/>
        <line x1="32" y1="86" x2="48" y2="86" class="wire"/>
        <text x="0" y="52" class="comp">Vin</text>
        <text x="0" y="112" class="value">{Vin}V</text>
        <line x1="40" y1="98" x2="40" y2="170" class="wire"/>
        <line x1="40" y1="62" x2="40" y2="15" class="wire"/>
        <line x1="40" y1="15" x2="90" y2="15" class="wire"/>
        

        <path d="M90,15 C95,0 105,0 110,15 C115,0 125,0 130,15 C135,0 145,0 150,15" class="wire"/>
        <text x="110" y="30" class="comp">L</text>
        <text x="90" y="40" class="value">{L_val}μH (RL={RL_val}Ω)</text>

        <line x1="150" y1="15" x2="210" y2="15" class="wire"/>
        <text x="200" y="10" class="label" font-size="9">node A</text>

        <line x1="210" y1="15" x2="210" y2="62" class="wire"/>
        <rect x="198" y="62" width="24" height="40" rx="3" style="stroke:#333;stroke-width:2;fill:#f0f4ff"/>
        <text x="203" y="82" class="comp" font-size="10">SW</text>
        <line x1="210" y1="102" x2="210" y2="160" class="wire"/>
        <text x="220" y="126" class="value">D={D_val:.2f}</text>
        <text x="220" y="140" class="value">f={f_val}kHz</text>

        <line x1="210" y1="15" x2="260" y2="15" class="wire"/>
        <polygon points="260,9 260,21 276,15" style="stroke:#333;stroke-width:2;fill:#fff8e1"/>
        <line x1="276" y1="9" x2="276" y2="21" class="wire"/>
        <line x1="276" y1="15" x2="330" y2="15" class="wire"/>
        <text x="270" y="30" class="comp">D</text>
        <text x="260" y="40" class="value">Vd={Vd_val}V</text>

        <line x1="330" y1="15" x2="330" y2="80" class="wire"/>
        <line x1="318" y1="80" x2="342" y2="80" class="wire"/>
        <line x1="318" y1="88" x2="342" y2="88" class="wire"/>
        <line x1="330" y1="88" x2="330" y2="160" class="wire"/>
        <text x="335" y="100" class="comp">C</text>
        <text x="335" y="110" class="value">{C_val}μF</text>

        <line x1="330" y1="15" x2="450" y2="15" class="wire"/>
        <line x1="450" y1="15" x2="450" y2="62" class="wire"/>
        <rect x="438" y="62" width="24" height="40" rx="2" style="stroke:#333;stroke-width:2;fill:#ffe0b2"/>
        <text x="405" y="80" class="comp">R</text>
        <text x="405" y="90" class="value">{R}Ω</text>
        <line x1="450" y1="102" x2="450" y2="160" class="wire"/>

        <circle cx="450" cy="15" r="3" style="fill:#333;"/>
        <text x="440" y="10" class="comp">Vout</text>
        <text x="470" y="10" class="value" style="fill:#1a73e8;font-weight:bold;font-size:11px;">{Vout_val:.2f}V</text>
        <text x="470" y="20" class="value" style="fill:#1a73e8;font-size:10px;">({mode_val})</text>
        
        <line x1="40" y1="170" x2="40" y2="170" class="wire"/>
        <line x1="35" y1="170" x2="45" y2="170" class="gnd"/>
        <line x1="37" y1="175" x2="43" y2="175" class="gnd"/>
        <line x1="39" y1="180" x2="41" y2="180" class="gnd"/>

        <line x1="40" y1="160" x2="450" y2="160" class="wire"/>
    </svg>
    '''

    return svg