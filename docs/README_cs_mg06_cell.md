# MG06 pracoviště s robotem AUBO i5 – MuJoCo model

Další verze projektu `mg06_loader_mujoco`. Na stojanu (deska `deska_robot`) stojí robot **AUBO i5** a na přírubě má stejný efektor jako v projektu MG06: **OnRobot VGC10 s vidlicí**. Model robota je z [ian-chuang/Manipulator-Mujoco](https://github.com/ian-chuang/Manipulator-Mujoco), licence MIT (`robot/aubo_i5/LICENSE_Manipulator-Mujoco`).

## Cyklus (`test_cell_cycle.py`, 65,8 s)
1. Robot vytáhne oranžovou DPS přísavkami VGC10 z levé modré přepravky (`crate_pick`, při pohledu zepředu).
2. Robot vloží DPS do otevřených čelistí stroje (výchozí pozice stroje), stroj čelisti zavře, robot vypne podtlak a odjede.
3. Stroj založí DPS do MG06 a zaparkuje (`y = −199,5 mm`, `z = +20 mm`).
4. Robot vidlicí MG06 zavře (víko dolů, madlo na 29°), odjede, vrátí se a MG06 otevře.
   - Zavření a otevření je stejná dráha příruby jako v projektu MG06, robot ji sleduje pomocí IK.
   - Odjezd a příjezd jsou nově naplánované podle skutečné polohy lišty po zavření. Viz „Odjezd od madla a příjezd k němu“.
5. Stroj DPS vezme z MG06 a vrátí se do výchozí pozice.
6. Robot DPS přísavkami převezme (stroj otevře čelisti) a uloží ji do pravé modré přepravky (`crate_place`). Konec cyklu.

## Spuštění
```
python test_cell_cycle.py                     # simulace celého cyklu + výpis (~1–2 min)
python tools/plan_cell.py                     # přeplánuje robota (IK + kontrola kolizí) -> cell_plan.npz
python cell.py                                # vygeneruje cell_scene.xml (celá buňka v jednom MJCF)
python -m mujoco.viewer --mjcf=cell_scene.xml # interaktivní prohlížení
```

## Soubory
- `cell.py` – sestavení buňky přes MjSpec:
  - scéna `scene.xml`, robot AUBO i5 na stojanu, VGC10 s vidlicí na přírubě
  - kolize přepravek jako dno a 4 stěny, polohové servo v kloubech robota
  - podtlak jako aktuátory `adhesion` na přísavkách, vzhled a kamery
- `test_cell_cycle.py` – fyzikální simulace celého cyklu podle plánu, s výpisem kontrol
- `cell_plan.npz` – hotový plán (trajektorie robota, pohyby stroje, události podtlaku)
- `tools/plan_cell.py` – výpočet plánu. Každý úsek robota se kontroluje na kolize se scénou ve stavu, jaký v tu chvíli bude.
- `robot_kin.py` – IK (damped least squares) pro TCP přísavek a přírubu, kontrola kolizí konfigurace robota
- `scene.xml` – hotová scéna: pracoviště, MG06, stroj (podavač), DPS
- `cell_scene.xml` – celá buňka včetně robota v jednom MJCF (pro prohlížeč)
- `pcb_meta.json` – pózy DPS (CAD, cíl založení, v čelistech) a polohy os stroje
- `mg06/assets/`, `machine/assets/`, `workstation/assets/` – meshe MG06, stroje a pracoviště
- `robot/aubo_i5/` – model robota z Manipulator-Mujoco (MJCF, meshe, licence)
- `robot_tool/` – VGC10 s vidlicí a zaznamenaná dráha zavření a otevření MG06 (`fork_close_open_path.npz`, potřebuje ji `plan_cell.py`)

Exportní a stavební skripty (Blender → meshe → `scene.xml`) a skripty pro video v tomto balíku nejsou. Jsou v předchozích verzích projektu (`mg06_loader_mujoco`, plné `mg06_cell_mujoco`).

## Robot
- Základna robota je na horní ploše desky `deska_robot`, což je počátek CAD souřadnic buňky. Natočení je −135°, takže všechny úlohy leží do ±65° od nuly prvního kloubu.
- **Rychlosti podle datasheetu AUBO i5:** max. 220°/s (J1–J4) a 230°/s (J5, J6).
  - Plán používá 60 % (`SPEED` v `plan_cell.py`), tj. 2,3–2,4 rad/s.
  - Zrychlení je omezené na 3 rad/s² (J1–J3) a 6 rad/s² (J4–J6). Datasheet zrychlení neuvádí, je to odhad.
  - Doba pohybu v kloubech je max(rychlostní, zrychlovací) limit pro profil minimum-jerk.
  - Volné lineární pohyby jedou 200–250 mm/s, posledních 15 mm k DPS 30 mm/s.
- **Pohony:** polohové servo v každém kloubu.
  - kp 5000/5000/4000/2000/2000/1500 Nm/rad, max. moment 180/180/150/60/60/60 Nm.
  - Armatura (redukovaná setrvačnost motoru a převodovky) 0,2/0,2/0,15/0,04/0,04/0,04 kg·m².
  - Kompenzace gravitace a dopředné řízení rychlosti.
  - Robot má proto skutečné zpoždění za trajektorií, max. 0,013 rad. Momenty a armatura jsou odhad, AUBO je nepublikuje.
- Manipulace s DPS používá konfiguraci „zápěstí nahoře“, dráha vidlice pro MG06 konfiguraci „zápěstí dole“. Přechod mezi nimi je ve volném prostoru přes průjezdní bod nad MG06.
- Plánovač při kontrole kolizí vyžaduje od robota i nesené DPS odstup aspoň 3 mm. Kde přímý pohyb v kloubech koliduje, vloží průjezdní bod nad středem dráhy.

## Přísavky (VGC10) – podtlak jako aktuátor `adhesion`
- **4 silikonové přísavky Ø15 mm při podtlaku 40 kPa** (datasheet VGC10: doporučeno 20–60 %).
  - Teoretická síla je 4 × π × 7,5² mm² × 40 kPa = 28 N, tj. 7 N na přísavku.
  - Každá přísavka je samostatné těleso s vlastním aktuátorem `adhesion` (`vacuum0..3`).
  - Síla působí jen při skutečném kontaktu přísavky s DPS. Když robot dosedne nakřivo nebo je DPS posunutá, drží jen přísavky, které se dotýkají, a úchop může selhat jako v reálu.
- **Měch přísavky:** každá přísavka je na posuvném kloubu s pružinou (zdvih 3 mm, 3000 N/m, tlumení), takže se přizpůsobí malému náklonu.
  - Robot přitlačí přísavky o 1,5 mm a pak zapne podtlak.
  - Do čelistí stroje vkládá DPS bez přítlaku, aby deska ležela nad osazeními prstů.
- Kontakt přísavka–DPS je kvádr se stejnou plochou jako přísavka (rovná plocha, čisté normály), tření 2,0, tuhé `solref 0.002`.
- Výsledek: obě uchopení mají v kontaktu všechny 4 přísavky.
- DPS se v přísavkách posune nejvýš o 1,3 mm. Asi 0,4 mm je poddajnost kontaktu při přenosu, zbytek je posun DPS čelistmi stroje při jejich zavírání (vystředění).
- Poloha TCP přísavek je 260,6 mm od příruby. DPS se přisává na horní plochu oblasti součástek (zjednodušení: součástky jsou v kolizích kvádr).

## Tenká DPS a kontakty
- Deska 1,8 mm je kvádr s `condim 4`, tuhým `solref 0.002 1`, `solimp 0.99 0.999` a `solmix 100`. Její parametry kontaktu tak převáží i proti měkčím protějškům. Časový krok je 0,5 ms.
- Test měří nejhlubší průnik DPS do okolí:
  - hnízdo MG06 0,20 mm, při vložení
  - čelisti stroje 0,11 mm
  - přepravka 0,07 mm
- Při položení DPS neposkakuje (náklon v hnízdě 0,02°).
- `margin`/`gap` nejsou použité: v MuJoCo 3.16 se `gap` nepromítá do `includemargin` a margin by nechal díly „vznášet“ 2–4 mm nad sebou.

## Mechanismus MG06
- MG06 je modelovaný jako skutečný mechanismus, ne jako přepínání stavů. Úhly víka a madla se v simulaci mění spojitě:
  - Víko má over-centre plynové pružiny (spatial tendon se `stiffness` a `springlength`): do −8° ho drží zavřené gravitace, nad tím ho pružiny drží otevřené.
  - Madlo má přepínací pružinu s mrtvým bodem ~31°, tření 0,6 Nm a tlumení. Rozsahy kloubů mají dorazy se `solreflimit`.
  - Přítlačná deska je svázaná s madlem vazbou `equality` (polynom 3. stupně).
  - Zámek je háček na madle a čep v základně (vlastní kolizní bity).
- Robot proto madlo opravdu tlačí a měří se síla na nástroji: **max. 294 N mezi vidlicí a lištou madla**, při dotlačení víka na doraz a madla na 29°.

## Stroj (podavač)
- Pohony os Y a Z mají kromě polohového serva i zpoždění 1. řádu (`dyntype filterexact`, τ = 30 ms). Osa tak cíl nedosáhne okamžitě: zpoždění za příkazem je při pohybu max. 23 mm, v cíli se dorovná.
- **Korekce montáže v ose X o +0,95 mm:**
  - V CADu je střed čelistí o 0,95 mm vlevo od středu hnízda a hnízdo nemá na šířku DPS vůli.
  - S tuhým kontaktem DPS dosedla na hranu levého zadního kolíku a zůstala nakřivo (1,4°).
  - Stroj je proto v modelu posunutý o 0,95 mm v X (`MACHINE_DX` při stavbě scény), jako by se seřídil při montáži. Polohy os se nemění.
- **Zadní pravá podpěra hnízda (`holder_10`):** v CADu je její horní plocha o 0,8 mm výš než ostatní tři podpěry.
  - Těžiště DPS leží mimo trojúhelník ostatních tří podpěr, deska se proto na tento roh vždy opře a bez úpravy by ležela nakřivo.
  - V modelu je podpěra snížená na úroveň ostatních (původní mesh je v plné verzi projektu).
  - Doporučuji zkontrolovat v CADu: buď je to chyba modelu, nebo má DPS v tom rohu výřez.

## Vzhled a kamery
- Neutrální podlaha s jemnou texturou a tři směrová světla, dvě s vrhanými stíny (`shadowsize 8192`).
- Materiály: hliníkové profily a kov se `specular` 0,75, plasty 0,25, tmavé díly 0,15. Odrazy (`reflectance`) jsou vypnuté kvůli rychlosti renderu.
- Kamery `cam_pcb` (sleduje DPS) a `cam_tool` (sleduje nástroj z druhé strany než robot), obě `mode="targetbody"`. V prohlížeči se přepínají klávesou `[` / `]`.

## Odjezd od madla a příjezd k němu
- Zaznamenaná dráha z projektu MG06 vyjíždí s lištou na dně drážky vidlice, takže je těsná: počítá s madlem přesně na 29,3°.
- S robotem zůstane madlo na 28,6° a víko se zvedne o 0,5° ve vůli zámku. Zub vidlice pak při odjezdu i příjezdu zavadil o lištu.
- Nová dráha:
  1. Vidlice se nejprve posune tak, aby lišta byla uprostřed drážky (vůle 5 mm na každou stranu) a 2 mm nade dnem.
  2. Pak vyjede podél drážky o 90 mm a 120 mm nahoru.
  3. Příjezd je stejná dráha pozpátku: 120 mm dolů a 90 mm podél drážky na lištu.
  4. Do zaznamenané dráhy otevírání se robot napojí až v místě, kde už madlo v záznamu míří nahoru pod skutečný úhel zavřeného madla. Stěna drážky tak lištu potká při pohybu nahoru a madlo se nikdy netlačí dolů (madlo stojí na 28,6°, dokud se nezačne otevírat).
- Při vysouvání a zasouvání je mezi vidlicí a lištou v simulaci 3,8–4,7 mm. Úhly madla a víka po zavření jsou v `plan_cell.py` (`H_CLOSED`, `LID_CLOSED`).

## Výsledek (`python test_cell_cycle.py`)
```
cycle time 65.8 s
  3.28s vacuum ON  (4 of 4 cups touch the PCB)
  9.52s vacuum OFF
 54.76s vacuum ON  (4 of 4 cups touch the PCB)
 62.44s vacuum OFF
PCB in MG06 (t=16.4s): error 0.30 mm (dx 0.04 dy -0.29 dz 0.06), tilt 0.02 deg
MG06 closed with the robot away (t=31.6s): lid -0.5 deg, handle 28.7 deg
PCB at the end: x 406.3 y -752.4 z 4.9 mm (right crate centre 406.2 -751.6, floor 4.0); tilt 0.00 deg
robot joint tracking error max 0.0126 rad, PCB slip in the cups max 1.32 mm, loader axis lag max 22.6 mm
max contact force fork -> handle bar: 294 N
deepest PCB penetration: MG06 holder -0.20 mm (t=16.14), crate -0.07 mm (t=62.46), loader jaws -0.11 mm (t=9.13)
robot <-> MG06 / workstation / crates             : no contact
robot tool <-> handle (outside closing / opening) : no contact
robot <-> loader                                  : no contact
loader <-> MG06 / workstation                     : no contact
PCB <-> lid / press plate                         : no contact
MG06 at the end: lid -40.0 deg, handle -0.0 deg
```
- Robot se nedotkne MG06, stroje, pracoviště ani přepravek. Vidlice se lišty madla dotýká jen při samotném zavírání a otevírání.
- DPS skončí v pravé přepravce do 1 mm od středu, rovně na dně.
- Pokud by simulace přestala být stabilní, test skončí chybou a neběží tiše znovu od nuly (MuJoCo při nestabilitě stav resetuje).

## Změny oproti mg06_loader_mujoco
- Výchozí stav stroje má čelisti otevřené (35 mm), DPS přináší robot.
- Zámek MG06: radiální vůle mezi háčkem a čepem je 1,5 mm místo 0,5 mm (`GAP` při stavbě MG06).
  - Robot tlačí víko dolů tužeji než volně vedená vidlice z minulé verze.
  - Víko se pak o 0,6 mm zaboří do základny, háček narazil na čep už při 27° a madlo se nedostalo na 29°.
- Kolize přepravek jsou dno a stěny místo plných konvexních obalů, takže robot může sahat dovnitř.

## Zjednodušení (robot)
- Podtlak má konstantní sílu na přísavku. Netěsnosti a průběh náběhu podtlaku (ventil, ejektor) se nesimulují.
- Momenty a armatura kloubů jsou odhad, zrychlení je zvolené (datasheet je neuvádí).
- Kabeláž a stojan robota (kromě desky) nejsou v kolizích.
- Měřítkem porovnání se skutečností je tedy hlavně struktura cyklu. Absolutní čas 65,8 s závisí na zvolených 60 % rychlosti a na zrychlení.

## Stroj, MG06 a pracoviště
- **Stroj (podavač):** dvě lineární osy a chapač.
  - QF12 S420, vodorovně k MG06: kloub `y_axis`, −199,5 … +220,5 mm
  - QF12 S300, svisle: kloub `z_axis`, −3,5 … +296,5 mm
  - SCHUNK EGI 040-EI: `jaw_L`/`jaw_R`, 0 … 40 mm na čelist, svázané rovností, síla 40 N na čelist
  - 0 = poloha v CADu.
- **Poloha stroje:**
  - Celý stroj je posunutý o 100 mm blíž k MG06, jinak chybí ~7,5 cm zdvihu. Přidaná je korekce +0,95 mm v X.
  - Výchozí pozice je `y = −100 mm`, `z = 0`: chapač je tam, kde je v CADu.
  - Založení do MG06 je v `y = +196,7 mm`, `z = +67,2 mm`. Parkování při zavírání MG06 je v `y = −199,5 mm`, `z = +20 mm`.
- **Úchop DPS:** prsty KS0010175 s osazením a přední doraz KS0010278, skloněné o 8,94° jako kazeta. DPS leží hranami na osazeních a svírá se předních ~38 mm.
- **MG06:** víko, madlo, přítlačná deska a zámek (viz „Mechanismus MG06“). DPS je samostatné těleso.
- **Pracoviště:** převzaté z `Test_it_off_-_Copy.blend` beze změny polohy. Rozvaděč pod stolem je v simulaci mělčí (600×600×200 místo 600×600×300), jinak by do něj motor svislé osy při založení vjel ~65 mm.
- **Zjednodušení:**
  - Hmotnosti stroje jsou odhad: Y vozík 4 kg, svislý vozík s chapačem 1,6 kg, DPS 60 g.
  - DPS je v kolizích kvádr desky, rohové pásky a kvádr součástek.
