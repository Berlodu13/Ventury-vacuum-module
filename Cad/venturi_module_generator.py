# venturi_module_generator.py
# Génère des STEP paramétriques du module Venturi (corps, porte-buse, capot)
# Dépendances: cadquery (pip install cadquery)

import cadquery as cq
from math import radians, cos, sin, sqrt

# ----------------------
# Paramètres (mm)
# ----------------------
body_outer_d = 24.0
conv_len = 10.0
throat_d = 4.0
throat_len = 5.0
diff_out_d = 8.0
diff_len = 40.0
inlet_stub_len = 8.0

vac_port_core_d = 6.2
vac_boss_d = 22.0
vac_boss_t = 10.0

gauge_pilot_d = 8.6  # pour taraudage 1/8" NPT
gauge_boss_d = 18.0
gauge_boss_t = 10.0

air_in_pilot_d = 11.8  # pour taraudage 1/4" BSP
exhaust_pilot_d = 11.8

wall = 4.0

nozzle_holder_od = 20.0
nozzle_holder_h = 22.0
nozzle_through_bore = 3.2
nozzle_seat_pilot = 5.2  # tarauder M6 ensuite

cap_od = 28.0
cap_h = 25.0
cap_core_d = 12.0
vent_d = 4.0

# Calculs dérivés
total_len = inlet_stub_len + conv_len + throat_len + diff_len + 6.0

# ----------------------
# Corps Venturi
# ----------------------
def make_venturi_body():
    # Corps extérieur
    body = cq.Workplane("XY").circle(body_outer_d / 2).extrude(total_len)

    # Canal axial par révolution du profil
    prof = (
        cq.Workplane("XZ")
        .moveTo(0, 0)
        .lineTo(inlet_stub_len, 0)  # stub de référence
        .lineTo(inlet_stub_len + conv_len, throat_d / 2)  # convergent
        .lineTo(inlet_stub_len + conv_len + throat_len, throat_d / 2)  # col
        .lineTo(inlet_stub_len + conv_len + throat_len + diff_len, diff_out_d / 2)  # diffuseur
        .lineTo(total_len, diff_out_d / 2)
        .lineTo(total_len, 0)
        .close()
    )
    canal = prof.revolve(360, (0, 0, 0), (0, 0, 1))
    body = body.cut(canal)

    # Perçages axiaux (échappement côté +Z, entrée air côté -Z)
    body = (
        body.faces(">Z").workplane(centerOption="CenterOfBoundBox").hole(exhaust_pilot_d, 12.0)
        .faces("<Z").workplane(centerOption="CenterOfBoundBox").hole(air_in_pilot_d, 12.0)
    )

    # Prise de vide (radiale X) au niveau du col
    throat_z_center = inlet_stub_len + conv_len + throat_len / 2.0

    # Bossage radial: esquisse dans YZ, extrusion le long de X, puis union
    vac_boss = (
        cq.Workplane("YZ")
        .workplane(origin=(0, 0, throat_z_center))
        .circle(vac_boss_d / 2.0)
        .extrude(vac_boss_t, both=True)  # extrusion le long de X
    )
    body = body.union(vac_boss)

    # Perçage radial (X)
    vac_cutter = (
        cq.Workplane("YZ")
        .workplane(origin=(0, 0, throat_z_center))
        .circle(vac_port_core_d / 2.0)
        .extrude(vac_boss_t + wall + 2.0, both=True)
    )
    body = body.cut(vac_cutter)

    # Port vacuomètre (radial Y), à +5 mm en aval du col
    gauge_z = throat_z_center + 5.0

    gauge_boss = (
        cq.Workplane("XZ")
        .workplane(origin=(0, 0, gauge_z))
        .circle(gauge_boss_d / 2.0)
        .extrude(gauge_boss_t, both=True)  # extrusion le long de Y
    )
    body = body.union(gauge_boss)

    gauge_cutter = (
        cq.Workplane("XZ")
        .workplane(origin=(0, 0, gauge_z))
        .circle(gauge_pilot_d / 2.0)
        .extrude(gauge_boss_t + wall + 2.0, both=True)
    )
    body = body.cut(gauge_cutter)

    # Chanfreins légers sur bords axiaux (désactivés pour éviter sélection vide / crash OCC)
    # Si tu veux les réactiver, fais-le sur une sélection contrôlée:
    # top_edges = body.faces(">Z").edges()
    # bottom_edges = body.faces("<Z").edges()
    # if top_edges.size() > 0:
    #     body = top_edges.chamfer(0.5)
    # if bottom_edges.size() > 0:
    #     body = bottom_edges.chamfer(0.5)

    return body

# ----------------------
# Porte-buse M6
# ----------------------
def make_nozzle_holder():
    nh = cq.Workplane("XY").circle(nozzle_holder_od / 2.0).extrude(nozzle_holder_h)

    # Alésage axial traversant
    nh = nh.faces(">Z").workplane().hole(nozzle_through_bore, nozzle_holder_h + 2.0)

    # Logement buse (pilote M6)
    nh = nh.faces(">Z").workplane().hole(nozzle_seat_pilot, 9.0)

    # Hexa de prise (clé 12, plat à plat)
    flat = 12.0
    # Rayon circonscrit pour un hexagone: R = F / sqrt(3), et polygon attend un diamètre
    hexa_diam = 2.0 * (flat / sqrt(3.0))
    hexa = (
        cq.Workplane("XY")
        .polygon(6, hexa_diam)
        .extrude(5.0)
        .translate((0, 0, nozzle_holder_h / 2.0))
    )
    nh = nh.union(hexa)

    # Chanfreins (facultatif; gardés simples)
    # nh = nh.edges("|Z").chamfer(0.5)

    return nh

# ----------------------
# Capot échappement silencé
# ----------------------
def make_exhaust_cap():
    cap = cq.Workplane("XY").circle(cap_od / 2.0).extrude(cap_h)
    cap = cap.faces(">Z").workplane().hole(cap_core_d, cap_h + 2.0)

    # Couronnes d'évents
    for ring, zf in enumerate([cap_h / 3.0, 2.0 * cap_h / 3.0]):
        offset = 0.0 if ring == 0 else 22.5
        for i in range(8):
            ang = radians(offset + i * 45.0)
            x = (cap_od / 2.0 - 4.0) * cos(ang)
            y = (cap_od / 2.0 - 4.0) * sin(ang)
            cap = cap.faces(">Z").workplane(origin=(x, y, zf)).hole(vent_d, 3.0, clean=True)

    # Chanfreins (désactivés pour stabilité)
    # cap = cap.edges("|Z").chamfer(0.5)

    return cap

# ----------------------
# Export STEP
# ----------------------
if __name__ == "__main__":
    body = make_venturi_body()
    holder = make_nozzle_holder()
    cap = make_exhaust_cap()

    cq.exporters.export(body, "venturi_body.step")
    cq.exporters.export(holder, "nozzle_holder.step")
    cq.exporters.export(cap, "exhaust_cap.step")

    print("STEP générés: venturi_body.step, nozzle_holder.step, exhaust_cap.step")
