import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import math
import time

# STM32'den bu süre boyunca (saniye) veri gelmezse robot anında durur
VERI_ZAMAN_ASIMI = 0.8


class MobilRobotSimulasyonu:
    def __init__(self, pencere):
        self.pencere = pencere
        self.pencere.title("STM32 Diferansiyel Mobil Robot Kontrol Paneli")
        self.pencere.configure(bg="#12141a")
        self.pencere.resizable(False, False)

        # Arka plan thread'i ile arayüz arasında veri güvenliği kilidi
        self.kilit = threading.Lock()

        # Seri port değişkenleri
        self.ser = None
        self.seri_thread = None
        self.okuma_aktif = False
        self.son_veri_zamani = 0.0
        self.paket_sayisi = 0
        self.son_paketler = []

        # Robot konum ve hız değerleri
        self.tuval_w = 640
        self.tuval_h = 480
        self.x = self.tuval_w / 2.0
        self.y = self.tuval_h / 2.0
        self.aci = 0.0
        self.toplam_mesafe = 0.0

        self.hedef_sol = 0
        self.hedef_sag = 0
        self.anlik_sol = 0
        self.anlik_sag = 0

        # Arayüz durumları
        self.duraklatildi = False
        self.iz_aktif = tk.BooleanVar(value=True)
        self.iz_cizgileri = []

        self.arayuzu_olustur()
        self.portlari_yenile()
        self.pencere.protocol("WM_DELETE_WINDOW", self.kapat)
        self.ekrani_guncelle()

    def arayuzu_olustur(self):
        ana = tk.Frame(self.pencere, bg="#12141a", padx=12, pady=12)
        ana.pack(fill="both", expand=True)

        # --- SOL TARAF: SİMÜLASYON SAHASI ---
        sol_panel = tk.Frame(ana, bg="#12141a")
        sol_panel.grid(row=0, column=0, padx=(0, 12))

        ust_bar = tk.Frame(sol_panel, bg="#12141a")
        ust_bar.pack(fill="x", pady=(0, 6))

        tk.Label(
            ust_bar,
            text="2D SİMÜLASYON SAHASI  (Işınlamak için sahaya tıkla)",
            fg="#8b949e", bg="#12141a", font=("Segoe UI", 9, "bold")
        ).pack(side="left")

        self.durum_rozeti = tk.Label(
            ust_bar, text="● BAĞLI DEĞİL",
            fg="#ff6b6b", bg="#26181c", font=("Consolas", 9, "bold"), padx=10, pady=2
        )
        self.durum_rozeti.pack(side="right")

        self.tuval = tk.Canvas(
            sol_panel, width=self.tuval_w, height=self.tuval_h,
            bg="#0d1117", highlightthickness=1, highlightbackground="#30363d", cursor="crosshair"
        )
        self.tuval.pack()
        self.tuval.bind("<Button-1>", self.robotu_tasi)
        self.izgara_ciz()

        # Robot çizimleri: 2 tekerlek + gövde + ön yön oku
        self.sol_teker_cizim = self.tuval.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#6e7681", outline="#0d1117")
        self.sag_teker_cizim = self.tuval.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#6e7681", outline="#0d1117")
        self.govde_cizim = self.tuval.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#1f6feb", outline="#58a6ff", width=2)
        self.on_ok_cizim = self.tuval.create_line(0, 0, 0, 0, fill="#ffd166", width=3, arrow=tk.LAST)

        # --- SAĞ TARAF: KONTROL VE TELEMETRİ ---
        sag_panel = tk.Frame(
            ana, bg="#161b22", width=280, padx=12, pady=12,
            highlightbackground="#30363d", highlightthickness=1
        )
        sag_panel.grid(row=0, column=1, sticky="ns")

        # 1. UART Bağlantısı
        self.baslik(sag_panel, "UART BAĞLANTISI")
        port_satir = tk.Frame(sag_panel, bg="#161b22")
        port_satir.pack(fill="x", pady=(4, 6))

        self.port_secim = ttk.Combobox(port_satir, width=10, state="readonly")
        self.port_secim.pack(side="left", padx=(0, 5))

        self.baud_secim = ttk.Combobox(
            port_satir, width=8, state="readonly", values=["115200", "9600", "57600", "38400"]
        )
        self.baud_secim.set("115200")
        self.baud_secim.pack(side="left", padx=(0, 5))

        tk.Button(
            port_satir, text="↻", command=self.portlari_yenile,
            bg="#21262d", fg="white", relief="flat", width=3, cursor="hand2"
        ).pack(side="left")

        self.baglan_btn = tk.Button(
            sag_panel, text="BAĞLAN", command=self.baglanti_degistir,
            bg="#238636", fg="white", font=("Segoe UI", 9, "bold"), relief="flat", pady=5, cursor="hand2"
        )
        self.baglan_btn.pack(fill="x", pady=(0, 12))

        # 2. Tekerlek Hız Barları
        self.baslik(sag_panel, "TEKERLEK HIZLARI (-50 / +50)")
        self.sol_hiz_lbl = tk.Label(sag_panel, text="Sol Teker: 0", fg="#c9d1d9", bg="#161b22", font=("Consolas", 10))
        self.sol_hiz_lbl.pack(anchor="w", pady=(4, 0))
        self.sol_bar = tk.Canvas(sag_panel, width=240, height=16, bg="#0d1117", highlightthickness=0)
        self.sol_bar.pack(fill="x", pady=(2, 6))

        self.sag_hiz_lbl = tk.Label(sag_panel, text="Sağ Teker: 0", fg="#c9d1d9", bg="#161b22", font=("Consolas", 10))
        self.sag_hiz_lbl.pack(anchor="w")
        self.sag_bar = tk.Canvas(sag_panel, width=240, height=16, bg="#0d1117", highlightthickness=0)
        self.sag_bar.pack(fill="x", pady=(2, 12))

        # 3. Telemetri
        self.baslik(sag_panel, "CANLI TELEMETRİ")
        self.telemetri_lbl = tk.Label(
            sag_panel,
            text="Konum : X=320  Y=240\nAçı   : 0°\nMesafe: 0 px\nPaket : 0",
            fg="#79c0ff", bg="#0d1117", font=("Consolas", 9),
            justify="left", anchor="w", padx=8, pady=6
        )
        self.telemetri_lbl.pack(fill="x", pady=(4, 12))

        # 4. Kontroller
        self.baslik(sag_panel, "KONTROLLER")
        hiz_satir = tk.Frame(sag_panel, bg="#161b22")
        hiz_satir.pack(fill="x", pady=(4, 6))
        tk.Label(hiz_satir, text="Hız Çarpanı:", fg="#8b949e", bg="#161b22", font=("Segoe UI", 9)).pack(side="left")
        self.hiz_slider = tk.Scale(
            hiz_satir, from_=0.3, to=2.5, resolution=0.1, orient="horizontal",
            bg="#161b22", fg="white", troughcolor="#0d1117", highlightthickness=0, length=145
        )
        self.hiz_slider.set(1.0)
        self.hiz_slider.pack(side="right")

        btn_grid = tk.Frame(sag_panel, bg="#161b22")
        btn_grid.pack(fill="x", pady=(0, 6))
        self.duraklat_btn = tk.Button(
            btn_grid, text="⏸ Duraklat", command=self.duraklat_degistir,
            bg="#9e6a03", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", pady=4, cursor="hand2"
        )
        self.duraklat_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        tk.Button(
            btn_grid, text="⌖ Merkeze Al", command=self.merkeze_al,
            bg="#21262d", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", pady=4, cursor="hand2"
        ).grid(row=0, column=1, sticky="ew")
        btn_grid.columnconfigure(0, weight=1)
        btn_grid.columnconfigure(1, weight=1)

        iz_satir = tk.Frame(sag_panel, bg="#161b22")
        iz_satir.pack(fill="x", pady=(2, 10))
        tk.Checkbutton(
            iz_satir, text="Hareket İzini Göster", variable=self.iz_aktif,
            bg="#161b22", fg="#c9d1d9", selectcolor="#0d1117",
            activebackground="#161b22", activeforeground="white", font=("Segoe UI", 9)
        ).pack(side="left")
        tk.Button(
            iz_satir, text="Temizle", command=self.izi_temizle,
            bg="#21262d", fg="#ff7b72", font=("Segoe UI", 8, "bold"), relief="flat", padx=8, cursor="hand2"
        ).pack(side="right")

        # 5. Canlı UART Monitör
        self.baslik(sag_panel, "GELEN UART VERİSİ (Thread)")
        self.log_lbl = tk.Label(
            sag_panel, text="Veri bekleniyor...",
            fg="#3fb950", bg="#0d1117", font=("Consolas", 8),
            justify="left", anchor="nw", height=4, padx=8, pady=4
        )
        self.log_lbl.pack(fill="x", pady=(4, 0))

    def baslik(self, parent, metin):
        tk.Label(parent, text=metin, fg="#8b949e", bg="#161b22", font=("Segoe UI", 8, "bold")).pack(anchor="w")

    def izgara_ciz(self):
        for x in range(0, self.tuval_w, 40):
            renk = "#1f2630" if x == self.tuval_w // 2 else "#151a22"
            self.tuval.create_line(x, 0, x, self.tuval_h, fill=renk)
        for y in range(0, self.tuval_h, 40):
            renk = "#1f2630" if y == self.tuval_h // 2 else "#151a22"
            self.tuval.create_line(0, y, self.tuval_w, y, fill=renk)

    def portlari_yenile(self):
        portlar = list(serial.tools.list_ports.comports())
        liste = [p.device for p in portlar]
        self.port_secim["values"] = liste
        secili = ""
        for p in portlar:
            if "STLink" in p.description or "STM" in p.description:
                secili = p.device
                break
        if not secili and liste:
            secili = liste[0]
        self.port_secim.set(secili)

    def baglanti_degistir(self):
        if self.okuma_aktif:
            self.baglantiyi_kes()
        else:
            self.baglan()

    def baglan(self):
        port = self.port_secim.get()
        baud = int(self.baud_secim.get())
        if not port:
            self.durum_rozeti.config(text="● PORT YOK", fg="#ff6b6b", bg="#26181c")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            self.okuma_aktif = True
            self.seri_thread = threading.Thread(target=self.seri_port_oku, daemon=True)
            self.seri_thread.start()
            self.baglan_btn.config(text="BAĞLANTIYI KES", bg="#da3633")
            self.durum_rozeti.config(text="● VERİ BEKLENİYOR", fg="#ffd166", bg="#2b2513")
        except Exception as hata:
            self.durum_rozeti.config(text="● PORT HATASI", fg="#ff6b6b", bg="#26181c")
            print("Bağlantı hatası:", hata)

    def baglantiyi_kes(self):
        self.okuma_aktif = False
        if self.seri_thread and self.seri_thread.is_alive():
            self.seri_thread.join(timeout=0.5)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        with self.kilit:
            self.hedef_sol = 0
            self.hedef_sag = 0
            self.son_veri_zamani = 0.0
        self.baglan_btn.config(text="BAĞLAN", bg="#238636")
        self.durum_rozeti.config(text="● BAĞLI DEĞİL", fg="#ff6b6b", bg="#26181c")

    def seri_port_oku(self):
        """Arka plan Thread'i: STM32'den gelen UART verilerini arayüzü kitlemeden okur."""
        while self.okuma_aktif and self.ser and self.ser.is_open:
            try:
                ham = self.ser.readline()
                if not ham:
                    continue
                satir = ham.decode("utf-8", errors="ignore").strip()
                if "," in satir:
                    parcalar = satir.split(",")
                    if len(parcalar) == 2:
                        sol = int(parcalar[0])
                        sag = int(parcalar[1])
                        with self.kilit:
                            self.hedef_sol = sol
                            self.hedef_sag = sag
                            self.son_veri_zamani = time.time()
                            self.paket_sayisi += 1
                            self.son_paketler.append(f"#{self.paket_sayisi:<4d} Sol:{sol:>4d} | Sağ:{sag:>4d}")
                            if len(self.son_paketler) > 4:
                                self.son_paketler.pop(0)
            except ValueError:
                continue
            except Exception as hata:
                print("Okuma koptu:", hata)
                self.okuma_aktif = False
                break

    def duraklat_degistir(self):
        self.duraklatildi = not self.duraklatildi
        if self.duraklatildi:
            self.duraklat_btn.config(text="▶ Devam Et", bg="#238636")
        else:
            self.duraklat_btn.config(text="⏸ Duraklat", bg="#9e6a03")

    def merkeze_al(self):
        self.x = self.tuval_w / 2.0
        self.y = self.tuval_h / 2.0
        self.aci = 0.0
        self.toplam_mesafe = 0.0
        self.izi_temizle()

    def izi_temizle(self):
        for cizgi in self.iz_cizgileri:
            self.tuval.delete(cizgi)
        self.iz_cizgileri.clear()

    def robotu_tasi(self, event):
        self.x = float(event.x)
        self.y = float(event.y)

    def nokta_dondur(self, dx, dy):
        c = math.cos(self.aci)
        s = math.sin(self.aci)
        return (self.x + dx * c - dy * s, self.y + dx * s + dy * c)

    def dikdortgen_koordinat(self, x1, y1, x2, y2):
        koseler = [
            self.nokta_dondur(x1, y1),
            self.nokta_dondur(x2, y1),
            self.nokta_dondur(x2, y2),
            self.nokta_dondur(x1, y2),
        ]
        return [k for nokta in koseler for k in nokta]

    def hiz_bari_ciz(self, bar_tuval, deger):
        bar_tuval.delete("all")
        w, h = 240, 16
        orta = w / 2
        bar_tuval.create_line(orta, 0, orta, h, fill="#30363d", width=2)
        if deger == 0:
            return
        oran = max(-1.0, min(1.0, deger / 50.0))
        uzunluk = oran * (w / 2 - 4)
        renk = "#3fb950" if deger > 0 else "#f85149"
        bar_tuval.create_rectangle(orta, 3, orta + uzunluk, h - 3, fill=renk, outline="")

    def teker_rengi(self, hiz):
        if hiz > 0:
            return "#3fb950"   # İleri: Yeşil
        if hiz < 0:
            return "#f85149"   # Geri: Kırmızı
        return "#6e7681"       # Durdu: Gri

    def ekrani_guncelle(self):
        simdi = time.time()

        with self.kilit:
            hedef_sol = self.hedef_sol
            hedef_sag = self.hedef_sag
            son_veri = self.son_veri_zamani
            paket = self.paket_sayisi
            log_metni = "\n".join(self.son_paketler) if self.son_paketler else "Veri bekleniyor..."

        veri_geliyor = self.okuma_aktif and (son_veri > 0) and ((simdi - son_veri) <= VERI_ZAMAN_ASIMI)

        if not self.okuma_aktif:
            self.anlik_sol = 0
            self.anlik_sag = 0
            self.durum_rozeti.config(text="● BAĞLI DEĞİL", fg="#ff6b6b", bg="#26181c")
        elif not veri_geliyor:
            # UART'tan veri kesildiği an robotu anında durdur
            self.anlik_sol = 0
            self.anlik_sag = 0
            self.durum_rozeti.config(text="● VERİ YOK - DURDU", fg="#ff6b6b", bg="#26181c")
        elif self.duraklatildi:
            self.anlik_sol = 0
            self.anlik_sag = 0
            self.durum_rozeti.config(text="● DURAKLATILDI", fg="#ffd166", bg="#2b2513")
        else:
            # Veri akıyorsa yumuşatma yapmadan direkt ham hızları bas (anlık tepki)
            self.anlik_sol = hedef_sol
            self.anlik_sag = hedef_sag
            self.durum_rozeti.config(text="● VERİ AKIYOR", fg="#3fb950", bg="#13291c")

        # Diferansiyel sürüş hareketi
        eski_x, eski_y = self.x, self.y
        olcek = self.hiz_slider.get()
        ileri_hiz = (self.anlik_sol + self.anlik_sag) / 2.0 * 0.16 * olcek
        donus_hizi = (self.anlik_sag - self.anlik_sol) * 0.004 * olcek

        self.aci += donus_hizi
        self.x += ileri_hiz * math.cos(self.aci)
        self.y += ileri_hiz * math.sin(self.aci)
        self.toplam_mesafe += abs(ileri_hiz)

        # Ekran kenarından çıkarsa karşı taraftan gir
        kenar_atladi = False
        if self.x < 0:
            self.x = self.tuval_w
            kenar_atladi = True
        elif self.x > self.tuval_w:
            self.x = 0
            kenar_atladi = True
        if self.y < 0:
            self.y = self.tuval_h
            kenar_atladi = True
        elif self.y > self.tuval_h:
            self.y = 0
            kenar_atladi = True

        # Hareket izi çiz
        if self.iz_aktif.get() and not kenar_atladi and (abs(ileri_hiz) > 0.05):
            cizgi = self.tuval.create_line(eski_x, eski_y, self.x, self.y, fill="#00e5ff", width=2)
            self.tuval.tag_lower(cizgi, self.sol_teker_cizim)
            self.iz_cizgileri.append(cizgi)
            if len(self.iz_cizgileri) > 350:
                self.tuval.delete(self.iz_cizgileri.pop(0))

        # Robot gövdesi ve tekerleklerini güncelle
        self.tuval.coords(self.sol_teker_cizim, *self.dikdortgen_koordinat(-12, -20, 12, -14))
        self.tuval.coords(self.sag_teker_cizim, *self.dikdortgen_koordinat(-12, 14, 12, 20))
        self.tuval.coords(self.govde_cizim, *self.dikdortgen_koordinat(-18, -14, 18, 14))

        on_uc = self.nokta_dondur(24, 0)
        self.tuval.coords(self.on_ok_cizim, self.x, self.y, on_uc[0], on_uc[1])

        self.tuval.itemconfig(self.sol_teker_cizim, fill=self.teker_rengi(self.anlik_sol))
        self.tuval.itemconfig(self.sag_teker_cizim, fill=self.teker_rengi(self.anlik_sag))

        # Sağ paneldeki göstergeleri güncelle
        self.sol_hiz_lbl.config(text=f"Sol Teker: {self.anlik_sol:>4d}")
        self.sag_hiz_lbl.config(text=f"Sağ Teker: {self.anlik_sag:>4d}")
        self.hiz_bari_ciz(self.sol_bar, self.anlik_sol)
        self.hiz_bari_ciz(self.sag_bar, self.anlik_sag)

        aci_derece = int(math.degrees(self.aci) % 360)
        self.telemetri_lbl.config(
            text=f"Konum : X={int(self.x):<3d}  Y={int(self.y):<3d}\n"
                 f"Açı   : {aci_derece}°\n"
                 f"Mesafe: {int(self.toplam_mesafe)} px\n"
                 f"Paket : {paket}"
        )
        self.log_lbl.config(text=log_metni)

        self.pencere.after(33, self.ekrani_guncelle)

    def kapat(self):
        self.baglantiyi_kes()
        self.pencere.destroy()


if __name__ == "__main__":
    ana_pencere = tk.Tk()
    MobilRobotSimulasyonu(ana_pencere)
    ana_pencere.mainloop()