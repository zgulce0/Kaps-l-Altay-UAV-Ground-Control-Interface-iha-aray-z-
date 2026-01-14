
from folium.elements import MacroElement
from jinja2 import Template

import threading
from pymavlink import mavutil



# Orange Cube Vehicle sınıfı
class OrangeCubeVehicle:
    def __init__(self, connection_string='tcp:127.0.0.1:5760', baud=57600):
        self.initialize_attributes()
        self.connection_string = connection_string
        self.baud = baud
        self.master = None
        self.is_connected = False
        self.data_lock = threading.Lock()

        # Drone verileri
        self.mode = type('Mode', (), {"name": "UNKNOWN"})()
        self.battery = type('Battery', (), {"level": 0, "voltage": 0.0, "current": 0.0})()
        self.gps_0 = type('GPS', (), {
            "fix_type": 0,
            "satellites_visible": 0,
            "eph": 0.0,  # GPS yatay doğruluk
            "epv": 0.0,  # GPS dikey doğruluk
            "vel": 0.0  # GPS hız doğruluğu
        })()
        self.location = type('Location', (), {
            "global_frame": type('GlobalFrame', (), {"lat": 0.0, "lon": 0.0, "alt": 0.0})(),
            "global_relative_frame": type('RelativeFrame', (), {"alt": 0.0})()
        })()
        self.attitude = type('Attitude', (), {
            "pitch": 0.0,
            "yaw": 0.0,
            "roll": 0.0,
            "rollspeed": 0.0,  # Roll açısal hızı
            "pitchspeed": 0.0,  # Pitch açısal hızı
            "yawspeed": 0.0  # Yaw açısal hızı
        })()
        self.velocity = type('Velocity', (), {"x": 0.0, "y": 0.0, "z": 0.0})()  # Değişiklik: obje olarak

        self.heading = 0
        self.armed = False
        self.is_armable = False
        self.groundspeed = 0.0

        # Ek flight controller verileri
        self.airspeed = 0.0
        self.throttle = 0
        self.climb_rate = 0.0
        self.current_waypoint = 0
        self.system_status = 0
        self.cpu_load = 0.0

        # Yeni veri yapıları (opsiyonel - gelişmiş özellikler için)
        self.rc_channels = {}
        self.servo_outputs = {}
        self.nav_controller = {}
        self.vibration = {}
        self.barometer = {}
        self.power_status = {}

        # Veri okuma thread'i
        self.data_thread = None
        self.running = False

    def connect(self):
        """Orange Cube'a bağlan"""
        try:
            print(f"Orange Cube bağlantısı deneniyor: {self.connection_string}")

            # MAVLink bağlantısını kur
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baud, timeout=5)

            # Heartbeat bekle (5 saniye timeout)
            print("Heartbeat bekleniyor...")
            heartbeat = self.master.wait_heartbeat(timeout=5)

            if heartbeat is None:
                raise Exception("Heartbeat alamadı - Orange Cube bağlı değil")

            print(
                f"✅ Heartbeat alındı - System: {self.master.target_system}, Component: {self.master.target_component}")

            self.is_connected = True

            # Veri okuma thread'ini başlat
            self.running = True
            self.data_thread = threading.Thread(target=self._read_data, daemon=True)
            self.data_thread.start()

            # İlk veri yüklemesini bekle
            time.sleep(1)

            return True

        except Exception as e:
            print(f"❌ Orange Cube bağlantı hatası: {e}")
            self.is_connected = False
            return False

    def _read_data(self):
        """Gerçek zamanlı veri okuma"""
        last_update = {}
        message_count = 0

        # Önemli mesaj tipleri - bu mesajlar daha sık güncellenebilir
        priority_messages = ['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'GPS_RAW_INT', 'SYS_STATUS']

        while self.running and self.is_connected:
            try:
                # FLUSH BUFFER KALDIRDIK! - Bu satırı kaldırın
                # self._flush_buffer()  # Bu satırı yoruma alın veya silin

                # Sadece yeni mesajları al
                msg = self.master.recv_match(blocking=True, timeout=0.05)

                if msg is None:
                    print('Timeout - mesaj bekleniyor...')
                    continue

                # Mesaj alındı
                message_count += 1
                msg_type = msg.get_type()
                current_time = time.time()

                # Debug için her 50 mesajda bir yazdır
                if message_count % 50 == 0:
                    print(f"Toplam {message_count} mesaj alındı")

                # Önemli mesajlar için daha sık güncelleme (50ms)
                # Diğer mesajlar için daha az sık güncelleme (200ms)
                min_interval = 0.05 if msg_type in priority_messages else 0.2

                # Aynı mesaj tipinden çok sık güncellemeler varsa atla
                if msg_type in last_update:
                    if current_time - last_update[msg_type] < min_interval:
                        continue

                last_update[msg_type] = current_time
                self._process_message(msg)

                # Debug için önemli mesajları yazdır
                if msg_type in ['HEARTBEAT', 'ATTITUDE', 'GPS_RAW_INT']:
                    print(f"✅ İşlenen mesaj: {msg_type}")

            except Exception as e:
                print(f"Veri okuma hatası: {e}")
                time.sleep(0.1)
                continue

    # Alternatif: Akıllı Buffer Temizleme
    def _smart_flush_buffer(self):
        """Sadece eski mesajları temizle, yenileri koru"""
        try:
            old_messages = 0
            # Sadece blocking=False ile bekleyen mesajları al
            while True:
                msg = self.master.recv_match(blocking=False, timeout=0)
                if msg is None:
                    break
                old_messages += 1
                # Maksimum 10 eski mesaj temizle, sonra dur
                if old_messages >= 10:
                    break

            if old_messages > 0:
                print(f"Eski {old_messages} mesaj temizlendi")
        except Exception as e:
            print(f"Buffer temizleme hatası: {e}")

    # Eğer buffer temizleme istiyorsanız, bu versiyonu kullanın:
    def _read_data_with_smart_flush(self):
        """Buffer'ı akıllı temizleyerek veri okuma"""
        last_update = {}
        message_count = 0

        priority_messages = ['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'GPS_RAW_INT', 'SYS_STATUS']

        while self.running and self.is_connected:
            try:
                # Sadece çok fazla mesaj birikirse temizle
                if message_count % 100 == 0 and message_count > 0:
                    self._smart_flush_buffer()

                msg = self.master.recv_match(blocking=True, timeout=1.0)

                if msg is None:
                    print('Timeout - yeni mesaj bekleniyor...')
                    continue

                message_count += 1
                msg_type = msg.get_type()
                current_time = time.time()

                # İstatistik
                if message_count % 100 == 0:
                    print(f"📊 Toplam {message_count} mesaj işlendi")

                # Mesaj filtreleme
                min_interval = 0.05 if msg_type in priority_messages else 0.2

                if msg_type in last_update:
                    if current_time - last_update[msg_type] < min_interval:
                        continue

                last_update[msg_type] = current_time
                self._process_message(msg)

                # Debug
                if msg_type in priority_messages:
                    print(f"✅ {msg_type} işlendi")

            except Exception as e:
                print(f"Veri okuma hatası: {e}")
                time.sleep(0.1)
                continue

    def _process_message(self, msg):
        """MAVLink mesajlarını işle"""
        with self.data_lock:
            try:
                msg_type = msg.get_type()

                if msg_type == 'ATTITUDE':
                    # Attitude bilgileri
                    self.attitude.roll = math.degrees(msg.roll)
                    self.attitude.pitch = math.degrees(msg.pitch)
                    self.attitude.yaw = math.degrees(msg.yaw)
                    self.heading = math.degrees(msg.yaw) % 360

                    # Ek attitude bilgileri
                    self.attitude.rollspeed = math.degrees(msg.rollspeed)
                    self.attitude.pitchspeed = math.degrees(msg.pitchspeed)
                    self.attitude.yawspeed = math.degrees(msg.yawspeed)

                elif msg_type == 'GPS_RAW_INT':
                    # GPS bilgileri
                    self.gps_0.fix_type = msg.fix_type
                    self.gps_0.satellites_visible = msg.satellites_visible
                    self.location.global_frame.lat = msg.lat / 1e7
                    self.location.global_frame.lon = msg.lon / 1e7
                    self.location.global_relative_frame.alt = msg.alt / 1000.0

                    # Ek GPS bilgileri
                    if hasattr(msg, 'eph'):
                        self.gps_0.eph = msg.eph / 100.0  # GPS yatay doğruluk
                    if hasattr(msg, 'epv'):
                        self.gps_0.epv = msg.epv / 100.0  # GPS dikey doğruluk
                    if hasattr(msg, 'vel'):
                        self.gps_0.vel = msg.vel / 100.0  # GPS hız doğruluğu

                elif msg_type == 'SYS_STATUS':
                    # Batarya ve sistem bilgileri
                    self.battery.voltage = msg.voltage_battery / 1000.0
                    self.battery.current = msg.current_battery / 100.0
                    self.battery.level = msg.battery_remaining

                    # Ek sistem bilgileri
                    self.system_status = msg.onboard_control_sensors_health
                    self.cpu_load = msg.load / 10.0  # CPU yükü %

                elif msg_type == 'HEARTBEAT':
                    # Heartbeat bilgileri
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.is_armable = not bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)

                    # Mod belirleme
                    custom_mode = msg.custom_mode
                    mode_mapping = {
                        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
                        5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
                        13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE"
                    }
                    self.mode.name = mode_mapping.get(custom_mode, f"CUSTOM_{custom_mode}")

                elif msg_type == 'VFR_HUD':
                    # VFR bilgileri
                    self.groundspeed = msg.groundspeed
                    self.airspeed = msg.airspeed
                    self.throttle = msg.throttle
                    self.climb_rate = msg.climb

                elif msg_type == 'GLOBAL_POSITION_INT':
                    # Detaylı konum bilgileri
                    self.location.global_frame.lat = msg.lat / 1e7
                    self.location.global_frame.lon = msg.lon / 1e7
                    self.location.global_frame.alt = msg.alt / 1000.0
                    self.location.global_relative_frame.alt = msg.relative_alt / 1000.0

                    # Hız bilgileri
                    if hasattr(self, 'velocity'):
                        self.velocity.x = msg.vx / 100.0
                        self.velocity.y = msg.vy / 100.0
                        self.velocity.z = msg.vz / 100.0

                    self.heading = msg.hdg / 100.0

                elif msg_type == 'RC_CHANNELS':
                    # RC kumanda bilgileri
                    if not hasattr(self, 'rc_channels'):
                        self.rc_channels = {}

                    self.rc_channels.update({
                        'roll': msg.chan1_raw,
                        'pitch': msg.chan2_raw,
                        'throttle': msg.chan3_raw,
                        'yaw': msg.chan4_raw,
                        'mode': msg.chan5_raw,
                        'aux1': msg.chan6_raw,
                        'aux2': msg.chan7_raw,
                        'aux3': msg.chan8_raw,
                    })

                elif msg_type == 'SERVO_OUTPUT_RAW':
                    # Motor/servo çıkışları
                    if not hasattr(self, 'servo_outputs'):
                        self.servo_outputs = {}

                    self.servo_outputs.update({
                        'motor1': msg.servo1_raw,
                        'motor2': msg.servo2_raw,
                        'motor3': msg.servo3_raw,
                        'motor4': msg.servo4_raw,
                        'servo1': msg.servo5_raw,
                        'servo2': msg.servo6_raw,
                        'servo3': msg.servo7_raw,
                        'servo4': msg.servo8_raw,
                    })

                elif msg_type == 'NAV_CONTROLLER_OUTPUT':
                    # Navigasyon kontrol bilgileri
                    if not hasattr(self, 'nav_controller'):
                        self.nav_controller = {}

                    self.nav_controller.update({
                        'nav_roll': msg.nav_roll,
                        'nav_pitch': msg.nav_pitch,
                        'nav_bearing': msg.nav_bearing,
                        'target_bearing': msg.target_bearing,
                        'wp_dist': msg.wp_dist,
                        'alt_error': msg.alt_error,
                        'aspd_error': msg.aspd_error,
                        'xtrack_error': msg.xtrack_error,
                    })

                elif msg_type == 'MISSION_CURRENT':
                    # Mevcut waypoint bilgisi
                    self.current_waypoint = msg.seq

                elif msg_type == 'VIBRATION':
                    # Vibrasyon bilgileri
                    if not hasattr(self, 'vibration'):
                        self.vibration = {}

                    self.vibration.update({
                        'x': msg.vibration_x,
                        'y': msg.vibration_y,
                        'z': msg.vibration_z,
                        'clipping_0': msg.clipping_0,
                        'clipping_1': msg.clipping_1,
                        'clipping_2': msg.clipping_2,
                    })

                elif msg_type == 'SCALED_PRESSURE':
                    # Barometrik basınç
                    if not hasattr(self, 'barometer'):
                        self.barometer = {}

                    self.barometer.update({
                        'pressure': msg.press_abs,  # hPa
                        'temperature': msg.temperature / 100.0,  # °C
                    })

                elif msg_type == 'POWER_STATUS':
                    # Güç durumu
                    if not hasattr(self, 'power_status'):
                        self.power_status = {}

                    self.power_status.update({
                        'vcc': msg.Vcc / 1000.0,  # V
                        'vservo': msg.Vservo / 1000.0,  # V
                        'flags': msg.flags,
                    })

            except Exception as e:
                print(f"Mesaj işleme hatası ({msg_type}): {e}")

    # Ayrıca bu yardımcı metodları da ekleyin:

    def initialize_attributes(self):
        """Sınıf özelliklerini başlat"""
        # Temel özellikler
        if not hasattr(self, 'velocity'):
            self.velocity = type('obj', (object,), {'x': 0, 'y': 0, 'z': 0})()

        if not hasattr(self, 'current_waypoint'):
            self.current_waypoint = 0

        if not hasattr(self, 'system_status'):
            self.system_status = 0

        if not hasattr(self, 'cpu_load'):
            self.cpu_load = 0

        # Diğer özellikler
        self.rc_channels = {}
        self.servo_outputs = {}
        self.nav_controller = {}
        self.vibration = {}
        self.barometer = {}
        self.power_status = {}

    def simple_goto(self, location, groundspeed=10):
        """Hedefe git komutu"""
        if not self.is_connected:
            print("❌ Orange Cube bağlı değil")
            return

        try:
            # MAVLink komutu gönder
            self.master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # type_mask
                int(location.lat * 1e7),  # lat_int
                int(location.lon * 1e7),  # lon_int
                location.alt,  # alt
                0, 0, 0,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz
                0, 0  # yaw, yaw_rate
            )
            print(f"[ORANGE_CUBE] Hedef konuma yönelme: {location.lat}, {location.lon} @ {groundspeed} m/s")
        except Exception as e:
            print(f"❌ Hedef gönderme hatası: {e}")

    def close(self):
        """Bağlantıyı kapat"""
        print("[ORANGE_CUBE] Bağlantı kapatılıyor...")
        self.running = False
        self.is_connected = False

        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=2)

        if self.master:
            self.master.close()

    def flush(self):
        """Buffer temizle"""
        if self.master:
            self.master.port.flush()


# --- Ui_MainWindow Sınıfı (Entegre Görseller Paneli ile) ---
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1400, 900)  # Görseller için daha geniş ve yüksek
        MainWindow.setWindowTitle("Kapsül Altay - İHA Kontrol Paneli")

        # İyileştirilmiş stil tanımlamaları - Daha renkli ve modern
        self.buttonStyle = """
               QPushButton {
                   background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                             stop: 0 #3498db, stop: 1 #2980b9);
                   color: white;
                   border-radius: 8px;
                   padding: 10px;
                   font-weight: bold;
                   font-size: 14px;
                   border: 1px solid #2980b9;
               }
               QPushButton:hover {
                   background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                             stop: 0 #3cb371, stop: 1 #2e8b57);
                   border: 1px solid #2e8b57;
               }
               QPushButton:pressed {
                   background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                             stop: 0 #27ae60, stop: 1 #229954);
               }
               QPushButton:disabled {
                   background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                             stop: 0 #95a5a6, stop: 1 #7f8c8d);
               }
           """

        self.panelStyle = """
               QGroupBox {
                   border: 2px solid #3498db;
                   border-radius: 12px;
                   background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                             stop: 0 #ffffff, stop: 1 #f8f9fa);
                   margin-top: 15px;
                   padding-top: 10px;
               }
               QGroupBox::title {
                   subcontrol-origin: margin;
                   left: 15px;
                   padding: 0 8px;
                   color: #2c3e50;
                   font-weight: bold;
                   font-size: 14px;
                   background-color: #ffffff;
               }
           """

        self.labelStyle = """
               QLabel {
                   font-size: 13px;
                   color: #2c3e50;
                   padding: 5px;
                   background-color: rgba(255, 255, 255, 0.8);
                   border-radius: 4px;
                   margin: 2px;
               }
           """

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        # Gradient arka plan
        self.centralwidget.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1,
                                          stop: 0 #e3f2fd, stop: 0.5 #f1f8e9, stop: 1 #fff3e0);
            }
        """)

        # Ana layout - Sol panel + Sağ panel (görseller)
        self.mainLayout = QtWidgets.QHBoxLayout(self.centralwidget)

        # Sol panel layout
        self.leftPanel = QtWidgets.QVBoxLayout()
        self.leftPanel.setContentsMargins(15, 15, 15, 15)

        # Logo ve başlık - daha göz alıcı
        self.logoLayout = QtWidgets.QHBoxLayout()
        self.logoLabel = QtWidgets.QLabel("🚁 KAPSÜL ALTAY")
        self.logoLabel.setStyleSheet("""
               font-size: 20px;
               font-weight: bold;
               color: #2c3e50;
               padding: 10px;
               background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,
                                         stop: 0 #3498db, stop: 1 #2ecc71);
               -webkit-background-clip: text;
               border-radius: 8px;
               border: 2px solid #3498db;
               background-color: #ffffff;
           """)
        self.logoLayout.addWidget(self.logoLabel)
        self.leftPanel.addLayout(self.logoLayout)

        # Bilgi Grubu
        self.infoGroup = QtWidgets.QGroupBox("📊 İHA Bilgileri")
        self.infoGroup.setStyleSheet(self.panelStyle)
        self.infoLayout = QtWidgets.QVBoxLayout(self.infoGroup)

        # Bilgi etiketleri (Batarya kısmı kaldırıldı) - renkli ikonlar
        self.infoLabels = {}
        info_data = [
            ("speed", "⚡ Hız: -- m/s"),
            ("altitude", "📏 İrtifa: -- m"),
            ("latitude", "🌍 GPS Enlem: --"),
            ("longitude", "🌎 GPS Boylam: --"),
            ("status", "🔗 Durum: Bağlantı Bekleniyor")
        ]

        for label_name, label_text in info_data:
            label = QtWidgets.QLabel(label_text)
            label.setStyleSheet(self.labelStyle)
            self.infoLabels[label_name] = label
            self.infoLayout.addWidget(label)

        # Komut Grubu - SADECE BAĞLAN VE BAĞLANTIYI KES BUTONLARI
        self.commandGroup = QtWidgets.QGroupBox("🎮 Komutlar")
        self.commandGroup.setStyleSheet(self.panelStyle)
        self.commandLayout = QtWidgets.QVBoxLayout(self.commandGroup)

        # Sadece temel butonları koru
        self.buttons = {}
        button_data = [
            ("connect", "🔗 Bağlan", "connect.png"),
            ("disconnect", "❌ Bağlantıyı Kes", "disconnect.png"),
        ]

        for btn_name, btn_text, btn_icon_file in button_data:
            buttonLayout = QtWidgets.QHBoxLayout()
            button = QtWidgets.QPushButton(btn_text)
            button.setStyleSheet(self.buttonStyle)
            button.setMinimumHeight(45)

            # İkon ekle (eğer dosya varsa)
            if os.path.exists(btn_icon_file):
                icon = QtGui.QIcon(btn_icon_file)
                button.setIcon(icon)

            self.buttons[btn_name] = button
            buttonLayout.addWidget(button)
            self.commandLayout.addLayout(buttonLayout)

            # Butonları fonksiyonlara bağla
            if btn_name == "connect":
                button.clicked.connect(self.connectDrone)
            elif btn_name == "disconnect":
                button.clicked.connect(self.disconnectDrone)

        # Başlangıçta disconnect butonunu devre dışı bırak
        self.buttons["disconnect"].setEnabled(False)

        # Telemetri Grubu
        self.telemetryGroup = QtWidgets.QGroupBox("📡 Telemetri")
        self.telemetryGroup.setStyleSheet(self.panelStyle)
        self.telemetryLayout = QtWidgets.QVBoxLayout(self.telemetryGroup)

        # Eğim, Yatış ve Yön Göstergeleri - renkli ikonlar
        telemetry_data = [
            ("attitude", "🔄 Eğim (Pitch): 0°"),
            ("roll", "🔃 Yatış (Roll): 0°"),
            ("heading", "🧭 Yön (Yaw): 0°"),
            ("vibration", "📳 Titreşim: 0.0 m/s²"),
            ("temperature", "🌡️ Sıcaklık: 25°C")
        ]

        telemetry_labels = {}
        for label_key, label_text in telemetry_data:
            label = QtWidgets.QLabel(label_text)
            label.setStyleSheet(self.labelStyle)
            telemetry_labels[label_key] = label
            self.telemetryLayout.addWidget(label)

        # Referansları sakla
        self.attitudeLabel = telemetry_labels["attitude"]
        self.rollLabel = telemetry_labels["roll"]
        self.headingLabel = telemetry_labels["heading"]
        self.vibrationLabel = telemetry_labels["vibration"]
        self.temperatureLabel = telemetry_labels["temperature"]

        # Sol panele grupları ekle
        self.leftPanel.addWidget(self.infoGroup)
        self.leftPanel.addWidget(self.commandGroup)
        self.leftPanel.addWidget(self.telemetryGroup)
        self.leftPanel.addStretch()

        # --- SAĞ PANEL: 2x2 GRİD DÜZEN İLE İYİLEŞTİRİLMİŞ GÖRSELLER PANELİ ---
        self.rightPanel = QtWidgets.QVBoxLayout()
        self.rightPanel.setContentsMargins(10, 10, 10, 10)

        # Görseller Grubu - 2x2 düzen için yeniden tasarlandı
        self.visualsGroup = QtWidgets.QGroupBox("✈️ Uçuş Göstergeleri")
        self.visualsGroup.setStyleSheet("""
               QGroupBox {
                   border: 3px solid #e74c3c;
                   border-radius: 15px;
                   background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                             stop: 0 #ffffff, stop: 1 #f8f9fa);
                   margin-top: 10px;
                   padding-top: 15px;
               }
               QGroupBox::title {
                   subcontrol-origin: margin;
                   left: 20px;
                   padding: 0 10px;
                   color: #e74c3c;
                   font-weight: bold;
                   font-size: 16px;
                   background-color: #ffffff;
               }
           """)
        self.visualsLayout = QtWidgets.QVBoxLayout(self.visualsGroup)
        self.visualsLayout.setContentsMargins(15, 20, 15, 15)
        self.visualsLayout.setSpacing(15)

        # 2x2 Grid layout ile göstergeleri düzenle
        self.gaugesGridLayout = QtWidgets.QGridLayout()
        self.gaugesGridLayout.setSpacing(15)
        self.gaugesGridLayout.setContentsMargins(10, 10, 10, 10)

        # HTML göstergeleri için web view'ları oluştur
        self.gauge_webviews = {}
        self.html_name_mapping = {}

        # HTML dosyalarını ve konumlarını tanımla - 2x3 düzen (6 gösterge)
        gauge_configs = [
            ("attitude", "attitude_gauge.html", 0, 0),  # Sol üst
            ("altimeter", "altimeter_gauge.html", 0, 1),  # Orta üst
            ("speed", "speed_gauge.html", 1, 1),  # Sağ üst

            ("vertical_speed", "vertical_speed_gauge.html", 1, 0),  # Orta alt

        ]

        for gauge_key, html_file, row, col in gauge_configs:
            # HTML dosyasının varlığını kontrol et
            if os.path.exists(html_file):
                # Web view container - daha büyük ve renkli
                gauge_container = QtWidgets.QFrame()

                # Farklı renkler her gösterge için
                colors = {
                    "attitude": {"border": "#e74c3c", "bg": "#ffebee"},
                    "altimeter": {"border": "#3498db", "bg": "#e3f2fd"},
                    "speed": {"border": "#f39c12", "bg": "#fff8e1"},
                    "heading": {"border": "#27ae60", "bg": "#e8f5e8"},
                    "vertical_speed": {"border": "#9b59b6", "bg": "#f3e5f5"},
                    "flight_instruments": {"border": "#e67e22", "bg": "#fdf2e9"}
                }

                color_scheme = colors.get(gauge_key, {"border": "#9b59b6", "bg": "#f3e5f5"})

                gauge_container.setStyleSheet(f"""
                       QFrame {{
                           background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                                     stop: 0 #ffffff, stop: 1 {color_scheme["bg"]});
                           border: 3px solid {color_scheme["border"]};
                           border-radius: 15px;
                           margin: 5px;
                       }}
                   """)
                # Daha büyük boyut - 2x3 için optimize edildi
                gauge_container.setFixedSize(380, 320)

                container_layout = QtWidgets.QVBoxLayout(gauge_container)
                container_layout.setContentsMargins(8, 8, 8, 8)
                container_layout.setSpacing(0)

                # Web view - 2x3 düzen için optimize edildi
                webview = QWebEngineView()
                webview.setMinimumSize(364, 304)
                webview.setMaximumSize(364, 304)

                # Web view'ın arka planını şeffaf yap
                webview.setStyleSheet("""
                       QWebEngineView {
                           background-color: transparent;
                           border: none;
                           border-radius: 10px;
                       }
                   """)

                # HTML dosyasını yükle
                webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(html_file)))

                # Container'a ekle
                container_layout.addWidget(webview)

                # Grid'e ekle
                self.gaugesGridLayout.addWidget(gauge_container, row, col)

                # Referansları sakla
                self.gauge_webviews[gauge_key] = webview
                self.html_name_mapping[gauge_key] = html_file

                print(f"Gösterge yüklendi: {html_file}")

        # Eğer hiç HTML dosyası bulunamazsa uyarı ekle
        if len(self.gauge_webviews) == 0:
            no_gauges_label = QtWidgets.QLabel(
                "❌ Hiç gösterge dosyası bulunamadı\n\n📁 Lütfen HTML gösterge dosyalarını kontrol edin:\n\n• attitude_gauge.html\n• altimeter_gauge.html\n• speed_gauge.html\n• heading_indicator.html\n• vertical_speed_gauge.html\n• flight_instruments.html")
            no_gauges_label.setStyleSheet("""
                   QLabel {
                       color: #dc3545;
                       font-size: 18px;
                       font-weight: bold;
                       text-align: center;
                       padding: 50px;
                       background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                                 stop: 0 #ffebee, stop: 1 #ffcdd2);
                       border: 3px solid #f44336;
                       border-radius: 15px;
                       margin: 20px;
                   }
               """)
            no_gauges_label.setAlignment(QtCore.Qt.AlignCenter)
            self.gaugesGridLayout.addWidget(no_gauges_label, 0, 0, 2, 3)

        # Grid'i ana layout'a ekle
        self.visualsLayout.addLayout(self.gaugesGridLayout)

        self.rightPanel.addWidget(self.visualsGroup)

        # Durum çubuğu - renkli
        self.statusBar = QtWidgets.QStatusBar()
        self.statusBar.setStyleSheet("""
            QStatusBar {
                background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,
                                          stop: 0 #3498db, stop: 1 #2ecc71);
                color: white;
                font-weight: bold;
                padding: 5px;
            }
        """)
        self.statusBar.showMessage(" Kapsül Altay İHA Kontrol Paneli Hazır")

        # Layout'ları ana layout'a ekle - oranları ayarla
        self.mainLayout.addLayout(self.leftPanel, 1)  # Sol panel %20
        self.mainLayout.addLayout(self.rightPanel, 4)  # Sağ panel %80

        MainWindow.setCentralWidget(self.centralwidget)
        MainWindow.setStatusBar(self.statusBar)

        # Menü bar oluştur - renkli
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.menuBar.setStyleSheet("""
            QMenuBar {
                background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                          stop: 0 #34495e, stop: 1 #2c3e50);
                color: white;
                font-weight: bold;
                padding: 5px;
            }
            QMenuBar::item {
                background: transparent;
                padding: 8px 12px;
                border-radius: 4px;
            }
            QMenuBar::item:selected {
                background: #3498db;
            }
            QMenu {
                background-color: #ffffff;
                border: 2px solid #3498db;
                border-radius: 8px;
                color: #2c3e50;
            }
            QMenu::item:selected {
                background-color: #3498db;
                color: white;
            }
        """)

        self.fileMenu = self.menuBar.addMenu(" Uçuş Verileri")

        # Menü öğeleri ekle
        self.saveAction = QtWidgets.QAction("💾 Uçuş Verilerini Kaydet", MainWindow)
        self.saveAction.setShortcut("Ctrl+S")
        self.fileMenu.addAction(self.saveAction)

        self.exitAction = QtWidgets.QAction("🚪 Çıkış", MainWindow)
        self.exitAction.setShortcut("Ctrl+Q")
        self.exitAction.triggered.connect(QtWidgets.qApp.quit)
        self.fileMenu.addAction(self.exitAction)

        MainWindow.setMenuBar(self.menuBar)

        # Başlangıç değerleri
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateDroneData)
        self.timer.start(100)

        self.flight_time = 0
        self.flight_timer = QtCore.QTimer()


        self.is_connected = False
        self.is_flying = False
        self.vehicle = None
    def update_all_gauges(self, pitch, roll, yaw, altitude=None, speed=None, heading=None, vertical_speed=None):
        """Tüm göstergeleri aynı anda günceller."""
        try:
            # Her gösterge için uygun JavaScript fonksiyonunu çağır
            for gauge_key, webview in self.gauge_webviews.items():
                if not webview.page().url().isValid():
                    continue

                if gauge_key == "attitude":
                    # Attitude göstergesi
                    webview.page().runJavaScript(f"updateAttitude({pitch}, {roll}, {yaw});")

                elif gauge_key == "altimeter":
                    # Altimeter göstergesi
                    alt_value = altitude if altitude is not None else pitch * 10
                    webview.page().runJavaScript(f"updateAltimeter({alt_value});")

                elif gauge_key == "speed":
                    # Hız göstergesi
                    speed_value = speed if speed is not None else abs(roll) * 5
                    webview.page().runJavaScript(f"updateSpeed({speed_value});")

                elif gauge_key == "heading":
                    # Yön göstergesi
                    heading_value = heading if heading is not None else yaw
                    webview.page().runJavaScript(f"updateHeading({heading_value});")

                elif gauge_key == "vertical_speed":
                    # Dikey hız göstergesi
                    vs_value = vertical_speed if vertical_speed is not None else 0
                    webview.page().runJavaScript(f"updateVerticalSpeed({vs_value});")
                    print("Alınan yükseklik değeri:", vs_value)

                elif gauge_key == "flight_instruments":
                    # Tüm uçuş enstrümanları
                    alt_val = altitude if altitude is not None else 0
                    speed_val = speed if speed is not None else 0
                    heading_val = heading if heading is not None else yaw
                    vs_val = vertical_speed if vertical_speed is not None else 0
                    webview.page().runJavaScript(
                        f"updateFlightInstruments({pitch}, {roll}, {yaw}, {alt_val}, {speed_val}, {heading_val}, {vs_val});")

        except Exception as e:
            print(f"Gösterge güncelleme hatası: {e}")


    def connectDrone(self):
        """Orange Cube'a bağlan"""
        try:
            print("Orange Cube bağlantısı deneniyor...")

            # --- GERÇEK ORANGE CUBE BAĞLANTISI ---
            # COM portunu kendi sisteminize göre ayarlayın (COM3, COM9, /dev/ttyUSB0 vs.)
            self.vehicle = OrangeCubeVehicle(connection_string='tcp:127.0.0.1:5762 ')

            # Bağlantıyı kur
            if not self.vehicle.connect():
                raise Exception("Orange Cube bağlantısı kurulamadı")

            # --- SİMÜLASYON İÇİN (TEST AMAÇLI) ---
            # Gerçek Orange Cube yoksa simülasyonu kullanın
            # self.vehicle = SimulatedVehicle()

            print("✅ Orange Cube bağlantısı KURULDU")

            # Bağlantı durumunu kontrol et
            if not self.vehicle.is_connected:
                raise Exception("Orange Cube bağlantısı doğrulanamadı")

            # Sistem bilgilerini yazdır
            time.sleep(0.5)  # Verilerin gelmesini bekle

            print(f"Mode: {self.vehicle.mode.name}")
            print(f"GPS: Fix Type {self.vehicle.gps_0.fix_type}, {self.vehicle.gps_0.satellites_visible} uydu")
            print(
                f"Location: Lat={self.vehicle.location.global_frame.lat:.6f}, Lon={self.vehicle.location.global_frame.lon:.6f}")
            print(f"Armed: {self.vehicle.armed}")

            self.is_connected = True
            self.is_flying = False
            self.flight_time = 0
            self.flight_timer.stop()

            # UI güncellemeleri
            self.statusBar.showMessage("Orange Cube bağlantısı başarılı")
            self.buttons["connect"].setEnabled(False)
            self.buttons["disconnect"].setEnabled(True)

            # Veri güncelleme timer'ını başlat
            self.timer.start(100)  # 10Hz güncelleme

        except Exception as e:
            print(f"❌ Orange Cube bağlantı hatası: {e}")
            self.statusBar.showMessage(f"Bağlantı hatası: {e}")
            self.is_connected = False

    def updateDroneData(self):
        """IHA verilerini güncelle"""
        # Eğer bağlı değilsek veya vehicle objesi yoksa simülasyon verisi göster
        if not self.is_connected or not hasattr(self, 'vehicle') or self.vehicle is None:
            # Simülasyon verileri üret
            sim_pitch = random.uniform(-25, 25)
            sim_roll = random.uniform(-30, 30)
            sim_yaw = random.uniform(0, 360)
            sim_altitude = random.uniform(0, 100)
            sim_speed = random.uniform(0, 50)
            sim_heading = random.uniform(0, 360)
            sim_vertical_speed = random.uniform(-10, 10)

            # Tüm göstergeleri simülasyon verileri ile güncelle
            self.update_all_gauges(sim_pitch, sim_roll, sim_yaw, sim_altitude, sim_speed, sim_heading,
                                   sim_vertical_speed)
            return

        v = self.vehicle

        try:
            # Data lock kullanarak thread-safe veri erişimi
            with v.data_lock:
                # HIZ BİLGİSİ
                speed = 0
                if hasattr(v, 'groundspeed') and v.groundspeed is not None:
                    speed = v.groundspeed
                elif hasattr(v, 'velocity'):
                    vx = getattr(v.velocity, 'x', 0) or 0
                    vy = getattr(v.velocity, 'y', 0) or 0
                    speed = math.sqrt(vx ** 2 + vy ** 2)

                # İRTİFA BİLGİSİ
                altitude = 0
                if hasattr(v, 'location'):
                    if hasattr(v.location, 'global_relative_frame'):
                        altitude = getattr(v.location.global_relative_frame, 'alt', 0) or 0
                    elif hasattr(v.location, 'global_frame'):
                        altitude = getattr(v.location.global_frame, 'alt', 0) or 0

                # GPS BİLGİSİ
                lat = 0
                lon = 0
                if hasattr(v, 'location') and hasattr(v.location, 'global_frame'):
                    lat = getattr(v.location.global_frame, 'lat', 0) or 0
                    lon = getattr(v.location.global_frame, 'lon', 0) or 0

                # ATTITUDE BİLGİSİ
                pitch = 0
                roll = 0
                yaw = 0
                if hasattr(v, 'attitude'):
                    pitch = getattr(v.attitude, 'pitch', 0) or 0
                    roll = getattr(v.attitude, 'roll', 0) or 0
                    yaw = getattr(v.attitude, 'yaw', 0) or 0

                # HEADING BİLGİSİ
                heading = getattr(v, 'heading', 0) or 0

                # DİKEY HIZ
                vertical_speed = 0
                if hasattr(v, 'velocity') and hasattr(v.velocity, 'z'):
                    vertical_speed = getattr(v.velocity, 'z', 0) or 0

                # UÇUŞ MODU VE ARM DURUMU
                flight_mode = "Bilinmiyor"
                armed_status = False
                if hasattr(v, 'mode') and hasattr(v.mode, 'name'):
                    flight_mode = getattr(v.mode, 'name', 'Bilinmiyor') or 'Bilinmiyor'
                armed_status = getattr(v, 'armed', False) or False

            # UI ELEMANLARINI GÜNCELLE
            if hasattr(self, 'infoLabels'):
                speed_kmh = speed * 3.6
                self.infoLabels["speed"].setText(f"Hız: {speed:.1f} m/s ({speed_kmh:.1f} km/h)")
                self.infoLabels["altitude"].setText(f"İrtifa: {altitude:.1f} m")
                self.infoLabels["latitude"].setText(f"GPS Enlem: {lat:.6f}")
                self.infoLabels["longitude"].setText(f"GPS Boylam: {lon:.6f}")

                arm_text = "Armlı" if armed_status else "Disarmlı"
                self.infoLabels["status"].setText(f"Durum: {flight_mode} ({arm_text})")

            # Telemetri bölümü
            if hasattr(self, 'attitudeLabel'):
                self.attitudeLabel.setText(f"Eğim (Pitch): {pitch:.1f}°")
            if hasattr(self, 'rollLabel'):
                self.rollLabel.setText(f"Yatış (Roll): {roll:.1f}°")
            if hasattr(self, 'headingLabel'):
                self.headingLabel.setText(f"Yön (Yaw): {yaw:.1f}° / Heading: {heading:.1f}°")

            # Sıcaklık ve titreşim
            temperature = 25
            if hasattr(v, 'barometer') and isinstance(v.barometer, dict):
                temperature = v.barometer.get('temperature', 25)
            if hasattr(self, 'temperatureLabel'):
                self.temperatureLabel.setText(f"Sıcaklık: {temperature:.1f}°C")

            if hasattr(self, 'vibrationLabel'):
                if hasattr(v, 'vibration') and isinstance(v.vibration, dict):
                    vib_x = v.vibration.get('x', 0)
                    vib_y = v.vibration.get('y', 0)
                    vib_z = v.vibration.get('z', 0)
                    vib_total = math.sqrt(vib_x ** 2 + vib_y ** 2 + vib_z ** 2)
                    self.vibrationLabel.setText(f"Titreşim: {vib_total:.2f} m/s²")
                else:
                    self.vibrationLabel.setText("Titreşim: Bilinmiyor")

            # Tüm göstergeleri güncelle
            self.update_all_gauges(pitch, roll, yaw, altitude, speed, heading, vertical_speed)

        except Exception as e:
            if hasattr(self, 'statusBar'):
                self.statusBar.showMessage(f"Veri güncelleme hatası: {e}")
            print(f"Veri güncelleme hatası: {e}")


    def disconnectDrone(self):
        """Orange Cube bağlantısını kes"""
        try:
            if self.vehicle:
                self.vehicle.close()

            self.is_connected = False
            self.is_flying = False
            self.timer.stop()
            self.flight_timer.stop()

            # UI güncellemeleri
            self.statusBar.showMessage("Orange Cube bağlantısı kesildi")
            self.buttons["connect"].setEnabled(True)
            self.buttons["disconnect"].setEnabled(False)

            print("✅ Orange Cube bağlantısı kesildi")

        except Exception as e:
            print(f"❌ Bağlantı kesme hatası: {e}")


if __name__ == "__main__":
    import sys
    import random
    import math
    import time
    import os
    from PyQt5 import QtCore, QtGui, QtWidgets
    from PyQt5.QtWebEngineWidgets import QWebEngineView

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())