# venturi_module_generator_v2.py
# Version améliorée avec STL, assembly, chanfreins sécurisés
# Dépendances : cadquery (pip install cadquery)

import cadquery as cq
from math import radians, cos, sin, sqrt
import os

# ----------------------
# Paramètres regroupés
# ----------------------
cfg = {
    "body_outer_d": 24.0,
    "conv_len": 10.0,
    "throat_d": 4.0,
    "throat_len": 5.0,
    "diff_out_d": 8.0,
    "diff_len": 40.0,
    "inlet_stub_len": 8.0,
    "vac_port_core_d": 6.2,
    "vac_boss_d": 22.0,
    "vac_boss_t": 10.0,
    "gauge_pilot_d": 8.6,  # taraudage 1/8" NPT
    "gauge_boss_d": 18.0,
    "gauge_boss_t": 10.0,
    "air_in_pilot_d": 11.8,  # 1/4" BSP
    "exhaust_pilot_d": 11.8,
    "wall": 4.0,
    "nozzle_holder_od": 20.0,
    "nozzle_holder_h": 22.0,
    "nozzle_through_bore": 3.2,
    "nozzle_seat_pilot": 5.2,  # M6 ensuite
    "cap_od": 28.0,
    "cap_h": 25.0,
    "cap_core_d": 12.0,
    "vent_d": 4.0,
}

# calculs dérivés
cfg["total_len"] = (
    cfg["inlet_stub_len"]
    + cfg["conv_len"]
    + cfg["throat_len"]
    + cfg["diff_len"]
    + 6.0
)

# ----------------------
# Fonctions utilitaires
# ----------------------
def safe_chamfer(obj, selector, dist=0.5):
    """Chamfreine si au moins une arête est trouvée"""
    edges = obj.edges(selector)
    return edges.chamfer(dist) if edges.size() > 0 else obj


# ----------------------
# Corps Venturi
# ----------------------
def make_venturi_body():
    p = cfg
    body = cq.Workplane("XY").circle(p["body_outer_d"] / 2).extrude(p["total_len"])

    # profil interne
    prof = (
        cq.Workplane("XZ")
        .moveTo(0, 0)
        .lineTo(p["inlet_stub_len"], 0)
        .lineTo(p["inlet_stub_len"] + p["conv_len"], p["throat_d"] / 2)
        .lineTo(p["inlet_stub_len"] + p["conv_len"] + p["throat_len"], p["throat_d"] / 2)
        .lineTo(p["inlet_stub_len"] + p["conv_len"] + p["throat_len"] + p["diff_len"], p["diff_out_d"] / 2)
        .lineTo(p["total_len"], p["diff_out_d"] / 2)
        .lineTo(p["total_len"], 0)
        .close()
    )
    canal = prof.revolve(360, (0, 0, 0), (0, 0, 1))
    body = body.cut(canal)

    # entrée / sortie
    body = (
        body.faces(">Z").workplane().hole(p["exhaust_pilot_d"], 12.0)
        .faces("<Z").workplane().hole(p["air_in_pilot_d"], 12.0)
    )

    # bossage radial (vide)
    throat_z_center = p["inlet_stub_len"] + p["conv_len"] + p["throat_len"] / 2.0
    vac_boss = (
        cq.Workplane("YZ")
        .workplane(origin=(0, 0, throat_z_center))
        .circle(p["vac_boss_d"] / 2.0)
        .extrude(p["vac_boss_t"], both=True)
    )
    body = body.union(vac_boss)
    vac_cutter = (
        cq.Workplane("YZ")
        .workplane(origin=(0, 0, throat_z_center))
        .circle(p["vac_port_core_d"] / 2.0)
        .extrude(p["vac_boss_t"] + p["wall"] + 2.0, both=True)
    )
    body = body.cut(vac_cutter)

    # bossage manomètre
    gauge_z = throat_z_center + 5.0
    gauge_boss = (
        cq.Workplane("XZ")
        .workplane(origin=(0, 0, gauge_z))
        .circle(p["gauge_boss_d"] / 2.0)
        .extrude(p["gauge_boss_t"], both=True)
    )
    body = body.union(gauge_boss)
    gauge_cutter = (
        cq.Workplane("XZ")
        .workplane(origin=(0, 0, gauge_z))
        .circle(p["gauge_pilot_d"] / 2.0)
        .extrude(p["gauge_boss_t"] + p["wall"] + 2.0, both=True)
    )
    body = body.cut(gauge_cutter)

    # chanfreins sécurisés
    body = safe_chamfer(body, ">Z", 0.5)
    body = safe_chamfer(body, "<Z", 0.5)
    return body


# ----------------------
# Porte-buse
# ----------------------
def make_nozzle_holder():
    p = cfg
    nh = cq.Workplane("XY").circle(p["nozzle_holder_od"] / 2.0).extrude(p["nozzle_holder_h"])
    nh = nh.faces(">Z").workplane().hole(p["nozzle_through_bore"], p["nozzle_holder_h"] + 2.0)
    nh = nh.faces(">Z").workplane().hole(p["nozzle_seat_pilot"], 9.0)

    # hexagone
    flat = 12.0
    hexa_diam = 2.0 * (flat / sqrt(3.0))
    hexa = cq.Workplane("XY").polygon(6, hexa_diam).extrude(5.0).translate((0, 0, p["nozzle_holder_h"] / 2.0))
    nh = nh.union(hexa)
    return nh


# ----------------------
# Capot échappement
# ----------------------
def make_exhaust_cap():
    p = cfg
    cap = cq.Workplane("XY").circle(p["cap_od"] / 2.0).extrude(p["cap_h"])
    cap = cap.faces(">Z").workplane().hole(p["cap_core_d"], p["cap_h"] + 2.0)

    # anneaux de trous
    for ring, zf in enumerate([p["cap_h"] / 3.0, 2.0 * p["cap_h"] / 3.0]):
        offset = 0.0 if ring == 0 else 22.5
        for i in range(8):
            ang = radians(offset + i * 45.0)
            x = (p["cap_od"] / 2.0 - 4.0) * cos(ang)
            y = (p["cap_od"] / 2.0 - 4.0) * sin(ang)
            cap = cap.faces(">Z").workplane(origin=(x, y, zf)).hole(p["vent_d"], 3.0, clean=True)
    return cap


# ----------------------
# Export
# ----------------------
if __name__ == "__main__":
    os.makedirs("out", exist_ok=True)
    body = make_venturi_body()
    holder = make_nozzle_holder()
    cap = make_exhaust_cap()

    # export STEP & STL
    cq.exporters.export(body, "out/venturi_body.step")
    cq.exporters.export(holder, "out/nozzle_holder.step")
    cq.exporters.export(cap, "out/exhaust_cap.step")
    cq.exporters.export(body, "out/venturi_body.stl")
    cq.exporters.export(holder, "out/nozzle_holder.stl")
    cq.exporters.export(cap, "out/exhaust_cap.stl")

    # assembly (STEP)
    assy = cq.Assembly()
    assy.add(body, name="body")
    assy.add(holder, name="nozzle_holder", loc=cq.Location((0, 0, -10)))  # position approximative
    assy.add(cap, name="cap", loc=cq.Location((0, 0, cfg["total_len"] + 5)))
    assy.save("out/venturi_assembly.step")

    print("✅ STEP + STL générés dans ./out/")
