from pptx import Presentation

prs = Presentation("Human_Following_Robot_Presentation.pptx")
print("Total slides:", len(prs.slides))
for idx, slide in enumerate(prs.slides, 1):
    title_txt = "(Title/Custom)"
    rects = []
    for sh in slide.shapes:
        l, t, w, h = sh.left / 914400, sh.top / 914400, sh.width / 914400, sh.height / 914400
        bottom, right = t + h, l + w
        if sh.has_text_frame and t > 0.15 and t < 0.7 and sh.text_frame.text.strip():
            title_txt = sh.text_frame.text.strip().replace("\n", " | ")[:80]
        if t > 1.1 and t < 7.0:
            rects.append((l, t, right, bottom, sh.shape_type, sh.name))
            if bottom > 7.05 or right > 13.25:
                print(f"  [WARN S{idx}] Shape {sh.name} extends past safe bounds: l={l:.2f} t={t:.2f} r={right:.2f} b={bottom:.2f}")
            if sh.has_text_frame:
                chars = len(sh.text_frame.text)
                area = w * h
                if chars > 0 and chars / max(area, 0.1) > 45:
                    print(f"  [DENSE S{idx}] chars={chars} area={area:.2f} density={chars/area:.1f} -> {sh.text_frame.text[:35]!r}")
        if str(sh.shape_type) in ("PICTURE (13)", "MEDIA (16)"):
            print(f"  [S{idx} MEDIA/PIC] l={l:.2f} t={t:.2f} w={w:.2f} h={h:.2f} r={right:.2f} b={bottom:.2f}")
        elif sh.has_table:
            print(f"  [S{idx} TABLE] l={l:.2f} t={t:.2f} w={w:.2f} h={h:.2f} r={right:.2f} b={bottom:.2f} rows={len(sh.table.rows)}")
    # Check pairwise overlaps among major non-text-on-card shapes
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            a, b = rects[i], rects[j]
            # ignore textbox sitting directly inside its own card
            if a[0] >= b[0] - 0.05 and a[2] <= b[2] + 0.05 and a[1] >= b[1] - 0.05 and a[3] <= b[3] + 0.05:
                continue
            if b[0] >= a[0] - 0.05 and b[2] <= a[2] + 0.05 and b[1] >= a[1] - 0.05 and b[3] <= a[3] + 0.05:
                continue
            ox = max(0, min(a[2], b[2]) - max(a[0], b[0]))
            oy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
            if ox > 0.05 and oy > 0.05:
                print(f"  [OVERLAP S{idx}] {a[5]} ({a[0]:.2f},{a[1]:.2f}-{a[2]:.2f},{a[3]:.2f}) vs {b[5]} ({b[0]:.2f},{b[1]:.2f}-{b[2]:.2f},{b[3]:.2f})")
    print(f"Slide {idx}: {title_txt}")
