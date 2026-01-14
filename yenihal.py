from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWebEngineWidgets import QWebEngineView
import folium
from folium.elements import MacroElement
from jinja2 import Template
import os
import sys
import random  # Simülasyon için
# from dronekit import connect, VehicleMode, LocationGlobalRelative # Dronekit'i kullanmak için yorum satırını kaldırın
import requests
import math
import threading
from pymavlink import mavutil
import time

# --- AttitudeWindow Sınıfı (Yeni Pencere) ---
class AttitudeWindow(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        class AttitudeWindow(QtWidgets.QWidget):
            def __init__(self, parent=None):
                super().__init__(parent)

                self.setWindowTitle("Detaylı Uçuş Göstergesi")
                self.setGeometry(100, 100, 1200, 800)  # Daha büyük pencere

                # Ana layout
                self.main_layout = QtWidgets.QVBoxLayout(self)

                # Üst kısım - HTML Göstergeler
                self.gauges_layout = QtWidgets.QHBoxLayout()

                # Attitude Gauge (Eğim Göstergesi)
                self.attitude_frame = QtWidgets.QFrame()
                self.attitude_frame.setFrameStyle(QtWidgets.QFrame.Box)
                self.attitude_layout = QtWidgets.QVBoxLayout(self.attitude_frame)
                self.attitude_label = QtWidgets.QLabel("Uçuş Verileri")
                self.attitude_label.setAlignment(QtCore.Qt.AlignCenter)
                self.attitude_label.setStyleSheet("font-weight: bold; font-size: 12px;")
                self.attitude_layout.addWidget(self.attitude_label)

                self.attitude_webview = QWebEngineView()
                self.attitude_webview.setFixedSize(300, 300)
                self.attitude_layout.addWidget(self.attitude_webview)

                # Attitude gauge HTML dosyasını yükle
                attitude_gauge_path = "attitude_gauge.html"
                if os.path.exists(attitude_gauge_path):
                    self.attitude_webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(attitude_gauge_path)))
                else:
                    print(f"Uyarı: {attitude_gauge_path} bulunamadı.")

                self.gauges_layout.addWidget(self.attitude_frame)

                # Altimeter Gauge (İrtifa Göstergesi)
                self.altimeter_frame = QtWidgets.QFrame()
                self.altimeter_frame.setFrameStyle(QtWidgets.QFrame.Box)
                self.altimeter_layout = QtWidgets.QVBoxLayout(self.altimeter_frame)
                self.altimeter_label = QtWidgets.QLabel("İrtifa Göstergesi")
                self.altimeter_label.setAlignment(QtCore.Qt.AlignCenter)
                self.altimeter_label.setStyleSheet("font-weight: bold; font-size: 12px;")
                self.altimeter_layout.addWidget(self.altimeter_label)

                self.altimeter_webview = QWebEngineView()
                self.altimeter_webview.setFixedSize(300, 300)
                self.altimeter_layout.addWidget(self.altimeter_webview)

                # Altimeter gauge HTML dosyasını yükle
                altimeter_gauge_path = "altimeter_gauge.html"
                if os.path.exists(altimeter_gauge_path):
                    self.altimeter_webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(altimeter_gauge_path)))
                else:
                    print(f"Uyarı: {altimeter_gauge_path} bulunamadı.")

                self.gauges_layout.addWidget(self.altimeter_frame)

                # Speed Gauge (Hız Göstergesi)
                self.speed_frame = QtWidgets.QFrame()
                self.speed_frame.setFrameStyle(QtWidgets.QFrame.Box)
                self.speed_layout = QtWidgets.QVBoxLayout(self.speed_frame)
                self.speed_label = QtWidgets.QLabel("Hız Göstergesi")
                self.speed_label.setAlignment(QtCore.Qt.AlignCenter)
                self.speed_label.setStyleSheet("font-weight: bold; font-size: 12px;")
                self.speed_layout.addWidget(self.speed_label)

                self.speed_webview = QWebEngineView()
                self.speed_webview.setFixedSize(300, 300)
                self.speed_layout.addWidget(self.speed_webview)

                # Speed gauge HTML dosyasını yükle
                speed_gauge_path = "speed_gauge.html"
                if os.path.exists(speed_gauge_path):
                    self.speed_webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(speed_gauge_path)))
                else:
                    print(f"Uyarı: {speed_gauge_path} bulunamadı.")

                self.gauges_layout.addWidget(self.speed_frame)

                # Speedometer (Hızölçer)
                self.speedometer_frame = QtWidgets.QFrame()
                self.speedometer_frame.setFrameStyle(QtWidgets.QFrame.Box)
                self.speedometer_layout = QtWidgets.QVBoxLayout(self.speedometer_frame)
                self.speedometer_label = QtWidgets.QLabel("Hızölçer")
                self.speedometer_label.setAlignment(QtCore.Qt.AlignCenter)
                self.speedometer_label.setStyleSheet("font-weight: bold; font-size: 12px;")
                self.speedometer_layout.addWidget(self.speedometer_label)

                self.speedometer_webview = QWebEngineView()
                self.speedometer_webview.setFixedSize(300, 300)
                self.speedometer_layout.addWidget(self.speedometer_webview)

                # Speedometer HTML dosyasını yükle
                speedometer_path = "speedometer.html"
                if os.path.exists(speedometer_path):
                    self.speedometer_webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(speedometer_path)))
                else:
                    print(f"Uyarı: {speedometer_path} bulunamadı.")

                self.gauges_layout.addWidget(self.speedometer_frame)

                self.main_layout.addLayout(self.gauges_layout)

                # Alt kısım - Kontrol paneli ve bilgiler
                self.bottom_layout = QtWidgets.QHBoxLayout()

                # Sol taraf - Kontrol butonları
                self.control_layout = QtWidgets.QVBoxLayout()

                # Arm/Disarm Butonları
                self.setup_arm_disarm_buttons()
                self.control_layout.addLayout(self.arm_disarm_layout)

                # Uçuş Bilgileri
                self.setup_flight_info()
                self.control_layout.addWidget(self.flight_info_group)

                self.bottom_layout.addLayout(self.control_layout)

                # Sağ taraf - Sistem durumu
                self.system_layout = QtWidgets.QVBoxLayout()
                self.setup_system_status()
                self.system_layout.addWidget(self.system_status_group)

                self.bottom_layout.addLayout(self.system_layout)

                self.main_layout.addLayout(self.bottom_layout)

                self.vehicle = None

                # Timer for periodic updates
                self.update_timer = QtCore.QTimer()
                self.update_timer.timeout.connect(self.update_all_displays)
                self.update_timer.start(100)  # 10 Hz update rate

            def setup_arm_disarm_buttons(self):
                """Arm/Disarm butonlarını ayarlar."""
                self.arm_disarm_layout = QtWidgets.QHBoxLayout()

                self.armButton = QtWidgets.QPushButton("ARM")
                self.armButton.setStyleSheet("""
                    QPushButton {
                        background-color: #2ecc71;
                        color: white;
                        border-radius: 4px;
                        padding: 15px;
                        font-weight: bold;
                        font-size: 16px;
                    }
                    QPushButton:hover { background-color: #27ae60; }
                    QPushButton:pressed { background-color: #1e8449; }
                    QPushButton:disabled { background-color: #bdc3c7; }
                """)
                self.armButton.clicked.connect(self.arm_drone)
                self.arm_disarm_layout.addWidget(self.armButton)

                self.disarmButton = QtWidgets.QPushButton("DISARM")
                self.disarmButton.setStyleSheet("""
                    QPushButton {
                        background-color: #e74c3c;
                        color: white;
                        border-radius: 4px;
                        padding: 15px;
                        font-weight: bold;
                        font-size: 16px;
                    }
                    QPushButton:hover { background-color: #c0392b; }
                    QPushButton:pressed { background-color: #a93226; }
                    QPushButton:disabled { background-color: #bdc3c7; }
                """)
                self.disarmButton.clicked.connect(self.disarm_drone)
                self.arm_disarm_layout.addWidget(self.disarmButton)

                self.armButton.setEnabled(False)
                self.disarmButton.setEnabled(False)

            def setup_flight_info(self):
                """Uçuş bilgilerini ayarlar."""
                self.flight_info_group = QtWidgets.QGroupBox("Uçuş Bilgileri")
                flight_layout = QtWidgets.QGridLayout(self.flight_info_group)

                # Eğim değerleri
                flight_layout.addWidget(QtWidgets.QLabel("Pitch:"), 0, 0)
                self.pitch_value_label = QtWidgets.QLabel("0.0°")
                self.pitch_value_label.setStyleSheet("font-weight: bold; font-size: 14px;")
                flight_layout.addWidget(self.pitch_value_label, 0, 1)

                flight_layout.addWidget(QtWidgets.QLabel("Roll:"), 1, 0)
                self.roll_value_label = QtWidgets.QLabel("0.0°")
                self.roll_value_label.setStyleSheet("font-weight: bold; font-size: 14px;")
                flight_layout.addWidget(self.roll_value_label, 1, 1)

                flight_layout.addWidget(QtWidgets.QLabel("Yaw:"), 2, 0)
                self.yaw_value_label = QtWidgets.QLabel("0.0°")
                self.yaw_value_label.setStyleSheet("font-weight: bold; font-size: 14px;")
                flight_layout.addWidget(self.yaw_value_label, 2, 1)

                # Hız bilgileri
                flight_layout.addWidget(QtWidgets.QLabel("Hava Hızı:"), 0, 2)
                self.airspeed_label = QtWidgets.QLabel("0.0 m/s")
                self.airspeed_label.setStyleSheet("font-weight: bold; font-size: 14px;")
                flight_layout.addWidget(self.airspeed_label, 0, 3)

                flight_layout.addWidget(QtWidgets.QLabel("Yer Hızı:"), 1, 2)
                self.groundspeed_label = QtWidgets.QLabel("0.0 m/s")
                self.groundspeed_label.setStyleSheet("font-weight: bold; font-size: 14px;")
                flight_layout.addWidget(self.groundspeed_label, 1, 3)

                flight_layout.addWidget(QtWidgets.QLabel("İrtifa:"), 2, 2)
                self.altitude_label = QtWidgets.QLabel("0.0 m")
                self.altitude_label.setStyleSheet("font-weight: bold; font-size: 14px;")
                flight_layout.addWidget(self.altitude_label, 2, 3)

            def setup_system_status(self):
                """Sistem durumu bilgilerini ayarlar."""
                self.system_status_group = QtWidgets.QGroupBox("Sistem Durumu")
                system_layout = QtWidgets.QGridLayout(self.system_status_group)

                # Batarya
                system_layout.addWidget(QtWidgets.QLabel("Batarya:"), 0, 0)
                self.battery_label = QtWidgets.QLabel("100%")
                self.battery_label.setStyleSheet("font-weight: bold; font-size: 14px; color: green;")
                system_layout.addWidget(self.battery_label, 0, 1)

                # Uçuş Modu
                system_layout.addWidget(QtWidgets.QLabel("Mod:"), 1, 0)
                self.flight_mode_label = QtWidgets.QLabel("STABILIZE")
                self.flight_mode_label.setStyleSheet("font-weight: bold; font-size: 14px;")
                system_layout.addWidget(self.flight_mode_label, 1, 1)

                # GPS Durumu
                system_layout.addWidget(QtWidgets.QLabel("GPS:"), 2, 0)
                self.gps_status_label = QtWidgets.QLabel("Aranıyor...")
                self.gps_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: orange;")
                system_layout.addWidget(self.gps_status_label, 2, 1)

                # GPS Koordinatları
                system_layout.addWidget(QtWidgets.QLabel("Enlem:"), 0, 2)
                self.latitude_label = QtWidgets.QLabel("0.000000")
                self.latitude_label.setStyleSheet("font-weight: bold; font-size: 12px;")
                system_layout.addWidget(self.latitude_label, 0, 3)

                system_layout.addWidget(QtWidgets.QLabel("Boylam:"), 1, 2)
                self.longitude_label = QtWidgets.QLabel("0.000000")
                self.longitude_label.setStyleSheet("font-weight: bold; font-size: 12px;")
                system_layout.addWidget(self.longitude_label, 1, 3)

                # Sinyal Gücü
                system_layout.addWidget(QtWidgets.QLabel("Sinyal:"), 2, 2)
                self.signal_strength_label = QtWidgets.QLabel("100%")
                self.signal_strength_label.setStyleSheet("font-weight: bold; font-size: 14px; color: green;")
                system_layout.addWidget(self.signal_strength_label, 2, 3)

            def set_vehicle(self, vehicle_instance):
                """Ana pencereden drone objesini alır."""
                self.vehicle = vehicle_instance
                self.update_arm_disarm_buttons()

            def update_all_displays(self):
                """Tüm göstergeleri günceller."""
                if not self.vehicle:
                    return

                try:
                    # Eğim değerlerini güncelle
                    if hasattr(self.vehicle, 'attitude'):
                        pitch = math.degrees(self.vehicle.attitude.pitch)
                        roll = math.degrees(self.vehicle.attitude.roll)
                        yaw = math.degrees(self.vehicle.attitude.yaw)

                        # Attitude gauge'ı güncelle
                        self.update_attitude_gauge(pitch, roll, yaw)

                        # Sayısal değerleri güncelle
                        self.pitch_value_label.setText(f"{pitch:.1f}°")
                        self.roll_value_label.setText(f"{roll:.1f}°")
                        self.yaw_value_label.setText(f"{yaw:.1f}°")

                    # Hız değerlerini güncelle
                    airspeed = 0
                    groundspeed = 0
                    if hasattr(self.vehicle, 'airspeed'):
                        airspeed = self.vehicle.airspeed
                        self.airspeed_label.setText(f"{airspeed:.1f} m/s")

                    if hasattr(self.vehicle, 'groundspeed'):
                        groundspeed = self.vehicle.groundspeed
                        self.groundspeed_label.setText(f"{groundspeed:.1f} m/s")

                    # Speed gauge'ları güncelle
                    self.update_speed_gauge(airspeed)
                    self.update_speedometer(groundspeed)

                    # İrtifa bilgilerini güncelle
                    altitude = 0
                    if hasattr(self.vehicle, 'location') and self.vehicle.location.global_relative_frame:
                        loc = self.vehicle.location.global_relative_frame
                        altitude = loc.alt
                        self.altitude_label.setText(f"{altitude:.1f} m")
                        self.latitude_label.setText(f"{loc.lat:.6f}")
                        self.longitude_label.setText(f"{loc.lon:.6f}")

                    # Altimeter gauge'ı güncelle
                    self.update_altimeter_gauge(altitude)

                    # Sistem durumu bilgilerini güncelle
                    if hasattr(self.vehicle, 'battery'):
                        battery_level = self.vehicle.battery.level if self.vehicle.battery else 100
                        self.battery_label.setText(f"{battery_level}%")

                        # Batarya renk kodlaması
                        if battery_level > 50:
                            color = "green"
                        elif battery_level > 20:
                            color = "orange"
                        else:
                            color = "red"
                        self.battery_label.setStyleSheet(f"font-weight: bold; font-size: 14px; color: {color};")

                    if hasattr(self.vehicle, 'mode'):
                        self.flight_mode_label.setText(str(self.vehicle.mode.name))

                    # GPS durumu
                    if hasattr(self.vehicle, 'gps_0'):
                        gps_fix = self.vehicle.gps_0.fix_type if self.vehicle.gps_0 else 0
                        if gps_fix >= 3:
                            self.gps_status_label.setText("3D Fix")
                            self.gps_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: green;")
                        elif gps_fix == 2:
                            self.gps_status_label.setText("2D Fix")
                            self.gps_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: orange;")
                        else:
                            self.gps_status_label.setText("No Fix")
                            self.gps_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: red;")

                    # Arm/Disarm butonlarını güncelle
                    self.update_arm_disarm_buttons()

                except Exception as e:
                    print(f"Display update error: {e}")

            def update_attitude_gauge(self, pitch, roll, yaw):
                """Attitude gauge'ı günceller."""
                if self.attitude_webview.page().url().isValid():
                    self.attitude_webview.page().runJavaScript(f"updateAttitude({pitch}, {roll}, {yaw});")

            def update_altimeter_gauge(self, altitude):
                """Altimeter gauge'ı günceller."""
                if self.altimeter_webview.page().url().isValid():
                    self.altimeter_webview.page().runJavaScript(f"updateAltimeter({altitude});")

            def update_speed_gauge(self, speed):
                """Speed gauge'ı günceller."""
                if self.speed_webview.page().url().isValid():
                    self.speed_webview.page().runJavaScript(f"updateSpeed({speed});")

            def update_speedometer(self, speed):
                """Speedometer'ı günceller."""
                if self.speedometer_webview.page().url().isValid():
                    self.speedometer_webview.page().runJavaScript(f"updateSpeedometer({speed});")

            def update_arm_disarm_buttons(self):
                """Arm/Disarm butonlarının durumunu günceller."""
                if self.vehicle:
                    is_armed = getattr(self.vehicle, 'armed', False)
                    self.armButton.setEnabled(not is_armed)
                    self.disarmButton.setEnabled(is_armed)
                else:
                    self.armButton.setEnabled(False)
                    self.disarmButton.setEnabled(False)

            def arm_drone(self):
                """Drone'u arm (kurma) komutu gönderir."""
                if not self.vehicle:
                    QtWidgets.QMessageBox.warning(self, "Hata", "Drone bağlı değil!")
                    return

                if not getattr(self.vehicle, 'is_armable', False):
                    QtWidgets.QMessageBox.warning(self, "Uyarı", "Drone şu anda arm edilebilir durumda değil!")
                    return

                msg = QtWidgets.QMessageBox()
                msg.setIcon(QtWidgets.QMessageBox.Question)
                msg.setText("Drone'u Arm etmek istediğinize emin misiniz?")
                msg.setInformativeText("Bu işlem pervanelerin dönmesine neden olacaktır!")
                msg.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

                if msg.exec_() == QtWidgets.QMessageBox.Yes:
                    try:
                        if isinstance(self.vehicle, OrangeCubeVehicle):
                            self.vehicle.armed = True
                            print("[SIM] Drone ARM edildi.")
                            QtWidgets.QMessageBox.information(self, "Başarılı", "Drone ARM edildi (Simülasyon).")
                        else:
                            print("Arming motors...")
                            # Gerçek DroneKit arm komutu burada olacak
                            print("Motors Armed!")
                            QtWidgets.QMessageBox.information(self, "Başarılı", "Drone ARM edildi.")

                        self.update_arm_disarm_buttons()
                    except Exception as e:
                        QtWidgets.QMessageBox.critical(self, "Hata", f"Arm hatası: {e}")

            def disarm_drone(self):
                """Drone'u disarm (devre dışı bırakma) komutu gönderir."""
                if not self.vehicle:
                    QtWidgets.QMessageBox.warning(self, "Hata", "Drone bağlı değil!")
                    return

                msg = QtWidgets.QMessageBox()
                msg.setIcon(QtWidgets.QMessageBox.Question)
                msg.setText("Drone'u Disarm etmek istediğinize emin misiniz?")
                msg.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

                if msg.exec_() == QtWidgets.QMessageBox.Yes:
                    try:
                        if isinstance(self.vehicle, OrangeCubeVehicle):
                            self.vehicle.armed = False
                            print("[SIM] Drone DISARM edildi.")
                            QtWidgets.QMessageBox.information(self, "Başarılı", "Drone DISARM edildi (Simülasyon).")
                        else:
                            print("Disarming motors...")
                            # Gerçek DroneKit disarm komutu burada olacak
                            print("Motors DISARMED!")
                            QtWidgets.QMessageBox.information(self, "Başarılı", "Drone DISARM edildi.")

                        self.update_arm_disarm_buttons()
                    except Exception as e:
                        QtWidgets.QMessageBox.critical(self, "Hata", f"Disarm hatası: {e}")

            def closeEvent(self, event):
                """Pencere kapanırken timer'ı durdur."""
                self.update_timer.stop()
                event.accept()
        self.setWindowTitle("Detaylı Uçuş Göstergesi")
        self.setGeometry(100, 100, 1200, 800)  # Daha büyük pencere

        # Ana layout
        self.main_layout = QtWidgets.QVBoxLayout(self)

        # Üst kısım - HTML Göstergeler
        self.gauges_layout = QtWidgets.QHBoxLayout()

        # Attitude Gauge (Eğim Göstergesi)
        self.attitude_frame = QtWidgets.QFrame()
        self.attitude_frame.setFrameStyle(QtWidgets.QFrame.Box)
        self.attitude_layout = QtWidgets.QVBoxLayout(self.attitude_frame)
        self.attitude_label = QtWidgets.QLabel("Uçuş Verileri")
        self.attitude_label.setAlignment(QtCore.Qt.AlignCenter)
        self.attitude_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        self.attitude_layout.addWidget(self.attitude_label)

        self.attitude_webview = QWebEngineView()
        self.attitude_webview.setFixedSize(300, 300)
        self.attitude_layout.addWidget(self.attitude_webview)

        # Attitude gauge HTML dosyasını yükle
        attitude_gauge_path = "attitude_gauge.html"
        if os.path.exists(attitude_gauge_path):
            self.attitude_webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(attitude_gauge_path)))
        else:
            print(f"Uyarı: {attitude_gauge_path} bulunamadı.")

        self.gauges_layout.addWidget(self.attitude_frame)

        # Altimeter Gauge (İrtifa Göstergesi)
        self.altimeter_frame = QtWidgets.QFrame()
        self.altimeter_frame.setFrameStyle(QtWidgets.QFrame.Box)
        self.altimeter_layout = QtWidgets.QVBoxLayout(self.altimeter_frame)
        self.altimeter_label = QtWidgets.QLabel("İrtifa Göstergesi")
        self.altimeter_label.setAlignment(QtCore.Qt.AlignCenter)
        self.altimeter_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        self.altimeter_layout.addWidget(self.altimeter_label)

        self.altimeter_webview = QWebEngineView()
        self.altimeter_webview.setFixedSize(300, 300)
        self.altimeter_layout.addWidget(self.altimeter_webview)

        # Altimeter gauge HTML dosyasını yükle
        altimeter_gauge_path = "altimeter_gauge.html"
        if os.path.exists(altimeter_gauge_path):
            self.altimeter_webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(altimeter_gauge_path)))
        else:
            print(f"Uyarı: {altimeter_gauge_path} bulunamadı.")

        self.gauges_layout.addWidget(self.altimeter_frame)

        # Speed Gauge (Hız Göstergesi)
        self.speed_frame = QtWidgets.QFrame()
        self.speed_frame.setFrameStyle(QtWidgets.QFrame.Box)
        self.speed_layout = QtWidgets.QVBoxLayout(self.speed_frame)
        self.speed_label = QtWidgets.QLabel("Hız Göstergesi")
        self.speed_label.setAlignment(QtCore.Qt.AlignCenter)
        self.speed_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        self.speed_layout.addWidget(self.speed_label)

        self.speed_webview = QWebEngineView()
        self.speed_webview.setFixedSize(300, 300)
        self.speed_layout.addWidget(self.speed_webview)

        # Speed gauge HTML dosyasını yükle
        speed_gauge_path = "speed_gauge.html"
        if os.path.exists(speed_gauge_path):
            self.speed_webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(speed_gauge_path)))
        else:
            print(f"Uyarı: {speed_gauge_path} bulunamadı.")

        self.gauges_layout.addWidget(self.speed_frame)

        # Speedometer (Hızölçer)
        self.speedometer_frame = QtWidgets.QFrame()
        self.speedometer_frame.setFrameStyle(QtWidgets.QFrame.Box)
        self.speedometer_layout = QtWidgets.QVBoxLayout(self.speedometer_frame)
        self.speedometer_label = QtWidgets.QLabel("Hızölçer")
        self.speedometer_label.setAlignment(QtCore.Qt.AlignCenter)
        self.speedometer_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        self.speedometer_layout.addWidget(self.speedometer_label)

        self.speedometer_webview = QWebEngineView()
        self.speedometer_webview.setFixedSize(300, 300)
        self.speedometer_layout.addWidget(self.speedometer_webview)

        # Speedometer HTML dosyasını yükle
        speedometer_path = "speedometer.html"
        if os.path.exists(speedometer_path):
            self.speedometer_webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(speedometer_path)))
        else:
            print(f"Uyarı: {speedometer_path} bulunamadı.")

        self.gauges_layout.addWidget(self.speedometer_frame)

        self.main_layout.addLayout(self.gauges_layout)

        # Alt kısım - Kontrol paneli ve bilgiler
        self.bottom_layout = QtWidgets.QHBoxLayout()

        # Sol taraf - Kontrol butonları
        self.control_layout = QtWidgets.QVBoxLayout()

        # Arm/Disarm Butonları
        self.setup_arm_disarm_buttons()
        self.control_layout.addLayout(self.arm_disarm_layout)

        # Uçuş Bilgileri
        self.setup_flight_info()
        self.control_layout.addWidget(self.flight_info_group)

        self.bottom_layout.addLayout(self.control_layout)

        # Sağ taraf - Sistem durumu
        self.system_layout = QtWidgets.QVBoxLayout()
        self.setup_system_status()
        self.system_layout.addWidget(self.system_status_group)

        self.bottom_layout.addLayout(self.system_layout)

        self.main_layout.addLayout(self.bottom_layout)

        self.vehicle = None

        # Timer for periodic updates
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_all_displays)
        self.update_timer.start(100)  # 10 Hz update rate

    def setup_arm_disarm_buttons(self):
        """Arm/Disarm butonlarını ayarlar."""
        self.arm_disarm_layout = QtWidgets.QHBoxLayout()

        self.armButton = QtWidgets.QPushButton("ARM")
        self.armButton.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                border-radius: 4px;
                padding: 15px;
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:hover { background-color: #27ae60; }
            QPushButton:pressed { background-color: #1e8449; }
            QPushButton:disabled { background-color: #bdc3c7; }
        """)
        self.armButton.clicked.connect(self.arm_drone)
        self.arm_disarm_layout.addWidget(self.armButton)

        self.disarmButton = QtWidgets.QPushButton("DISARM")
        self.disarmButton.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border-radius: 4px;
                padding: 15px;
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:hover { background-color: #c0392b; }
            QPushButton:pressed { background-color: #a93226; }
            QPushButton:disabled { background-color: #bdc3c7; }
        """)
        self.disarmButton.clicked.connect(self.disarm_drone)
        self.arm_disarm_layout.addWidget(self.disarmButton)

        self.armButton.setEnabled(False)
        self.disarmButton.setEnabled(False)

    def setup_flight_info(self):
        """Uçuş bilgilerini ayarlar."""
        self.flight_info_group = QtWidgets.QGroupBox("Uçuş Bilgileri")
        flight_layout = QtWidgets.QGridLayout(self.flight_info_group)

        # Eğim değerleri
        flight_layout.addWidget(QtWidgets.QLabel("Pitch:"), 0, 0)
        self.pitch_value_label = QtWidgets.QLabel("0.0°")
        self.pitch_value_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        flight_layout.addWidget(self.pitch_value_label, 0, 1)

        flight_layout.addWidget(QtWidgets.QLabel("Roll:"), 1, 0)
        self.roll_value_label = QtWidgets.QLabel("0.0°")
        self.roll_value_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        flight_layout.addWidget(self.roll_value_label, 1, 1)

        flight_layout.addWidget(QtWidgets.QLabel("Yaw:"), 2, 0)
        self.yaw_value_label = QtWidgets.QLabel("0.0°")
        self.yaw_value_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        flight_layout.addWidget(self.yaw_value_label, 2, 1)

        # Hız bilgileri
        flight_layout.addWidget(QtWidgets.QLabel("Hava Hızı:"), 0, 2)
        self.airspeed_label = QtWidgets.QLabel("0.0 m/s")
        self.airspeed_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        flight_layout.addWidget(self.airspeed_label, 0, 3)

        flight_layout.addWidget(QtWidgets.QLabel("Yer Hızı:"), 1, 2)
        self.groundspeed_label = QtWidgets.QLabel("0.0 m/s")
        self.groundspeed_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        flight_layout.addWidget(self.groundspeed_label, 1, 3)

        flight_layout.addWidget(QtWidgets.QLabel("İrtifa:"), 2, 2)
        self.altitude_label = QtWidgets.QLabel("0.0 m")
        self.altitude_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        flight_layout.addWidget(self.altitude_label, 2, 3)

    def setup_system_status(self):
        """Sistem durumu bilgilerini ayarlar."""
        self.system_status_group = QtWidgets.QGroupBox("Sistem Durumu")
        system_layout = QtWidgets.QGridLayout(self.system_status_group)

        # Batarya
        system_layout.addWidget(QtWidgets.QLabel("Batarya:"), 0, 0)
        self.battery_label = QtWidgets.QLabel("100%")
        self.battery_label.setStyleSheet("font-weight: bold; font-size: 14px; color: green;")
        system_layout.addWidget(self.battery_label, 0, 1)

        # Uçuş Modu
        system_layout.addWidget(QtWidgets.QLabel("Mod:"), 1, 0)
        self.flight_mode_label = QtWidgets.QLabel("STABILIZE")
        self.flight_mode_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        system_layout.addWidget(self.flight_mode_label, 1, 1)

        # GPS Durumu
        system_layout.addWidget(QtWidgets.QLabel("GPS:"), 2, 0)
        self.gps_status_label = QtWidgets.QLabel("Aranıyor...")
        self.gps_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: orange;")
        system_layout.addWidget(self.gps_status_label, 2, 1)

        # GPS Koordinatları
        system_layout.addWidget(QtWidgets.QLabel("Enlem:"), 0, 2)
        self.latitude_label = QtWidgets.QLabel("0.000000")
        self.latitude_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        system_layout.addWidget(self.latitude_label, 0, 3)

        system_layout.addWidget(QtWidgets.QLabel("Boylam:"), 1, 2)
        self.longitude_label = QtWidgets.QLabel("0.000000")
        self.longitude_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        system_layout.addWidget(self.longitude_label, 1, 3)

        # Sinyal Gücü
        system_layout.addWidget(QtWidgets.QLabel("Sinyal:"), 2, 2)
        self.signal_strength_label = QtWidgets.QLabel("100%")
        self.signal_strength_label.setStyleSheet("font-weight: bold; font-size: 14px; color: green;")
        system_layout.addWidget(self.signal_strength_label, 2, 3)

    def set_vehicle(self, vehicle_instance):
        """Ana pencereden drone objesini alır."""
        self.vehicle = vehicle_instance
        self.update_arm_disarm_buttons()

    def update_all_displays(self):
        """Tüm göstergeleri günceller."""
        if not self.vehicle:
            return

        try:
            # Eğim değerlerini güncelle
            if hasattr(self.vehicle, 'attitude'):
                pitch = math.degrees(self.vehicle.attitude.pitch)
                roll = math.degrees(self.vehicle.attitude.roll)
                yaw = math.degrees(self.vehicle.attitude.yaw)

                # Attitude gauge'ı güncelle
                self.update_attitude_gauge(pitch, roll, yaw)

                # Sayısal değerleri güncelle
                self.pitch_value_label.setText(f"{pitch:.1f}°")
                self.roll_value_label.setText(f"{roll:.1f}°")
                self.yaw_value_label.setText(f"{yaw:.1f}°")

            # Hız değerlerini güncelle
            airspeed = 0
            groundspeed = 0
            if hasattr(self.vehicle, 'airspeed'):
                airspeed = self.vehicle.airspeed
                self.airspeed_label.setText(f"{airspeed:.1f} m/s")

            if hasattr(self.vehicle, 'groundspeed'):
                groundspeed = self.vehicle.groundspeed
                self.groundspeed_label.setText(f"{groundspeed:.1f} m/s")

            # Speed gauge'ları güncelle
            self.update_speed_gauge(airspeed)
            self.update_speedometer(groundspeed)

            # İrtifa bilgilerini güncelle
            altitude = 0
            if hasattr(self.vehicle, 'location') and self.vehicle.location.global_relative_frame:
                loc = self.vehicle.location.global_relative_frame
                altitude = loc.alt
                self.altitude_label.setText(f"{altitude:.1f} m")
                self.latitude_label.setText(f"{loc.lat:.6f}")
                self.longitude_label.setText(f"{loc.lon:.6f}")

            # Altimeter gauge'ı güncelle
            self.update_altimeter_gauge(altitude)

            # Sistem durumu bilgilerini güncelle
            if hasattr(self.vehicle, 'battery'):
                battery_level = self.vehicle.battery.level if self.vehicle.battery else 100
                self.battery_label.setText(f"{battery_level}%")

                # Batarya renk kodlaması
                if battery_level > 50:
                    color = "green"
                elif battery_level > 20:
                    color = "orange"
                else:
                    color = "red"
                self.battery_label.setStyleSheet(f"font-weight: bold; font-size: 14px; color: {color};")

            if hasattr(self.vehicle, 'mode'):
                self.flight_mode_label.setText(str(self.vehicle.mode.name))

            # GPS durumu
            if hasattr(self.vehicle, 'gps_0'):
                gps_fix = self.vehicle.gps_0.fix_type if self.vehicle.gps_0 else 0
                if gps_fix >= 3:
                    self.gps_status_label.setText("3D Fix")
                    self.gps_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: green;")
                elif gps_fix == 2:
                    self.gps_status_label.setText("2D Fix")
                    self.gps_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: orange;")
                else:
                    self.gps_status_label.setText("No Fix")
                    self.gps_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: red;")

            # Arm/Disarm butonlarını güncelle
            self.update_arm_disarm_buttons()

        except Exception as e:
            print(f"Display update error: {e}")

    def update_attitude_gauge(self, pitch, roll, yaw):
        """Attitude gauge'ı günceller."""
        if self.attitude_webview.page().url().isValid():
            self.attitude_webview.page().runJavaScript(f"updateAttitude({pitch}, {roll}, {yaw});")

    def update_altimeter_gauge(self, altitude):
        """Altimeter gauge'ı günceller."""
        if self.altimeter_webview.page().url().isValid():
            self.altimeter_webview.page().runJavaScript(f"updateAltimeter({altitude});")

    def update_speed_gauge(self, speed):
        """Speed gauge'ı günceller."""
        if self.speed_webview.page().url().isValid():
            self.speed_webview.page().runJavaScript(f"updateSpeed({speed});")

    def update_speedometer(self, speed):
        """Speedometer'ı günceller."""
        if self.speedometer_webview.page().url().isValid():
            self.speedometer_webview.page().runJavaScript(f"updateSpeedometer({speed});")

    def update_arm_disarm_buttons(self):
        """Arm/Disarm butonlarının durumunu günceller."""
        if self.vehicle:
            is_armed = getattr(self.vehicle, 'armed', False)
            self.armButton.setEnabled(not is_armed)
            self.disarmButton.setEnabled(is_armed)
        else:
            self.armButton.setEnabled(False)
            self.disarmButton.setEnabled(False)

    def arm_drone(self):
        """Drone'u arm (kurma) komutu gönderir."""
        if not self.vehicle:
            QtWidgets.QMessageBox.warning(self, "Hata", "Drone bağlı değil!")
            return

        if not getattr(self.vehicle, 'is_armable', False):
            QtWidgets.QMessageBox.warning(self, "Uyarı", "Drone şu anda arm edilebilir durumda değil!")
            return

        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Question)
        msg.setText("Drone'u Arm etmek istediğinize emin misiniz?")
        msg.setInformativeText("Bu işlem pervanelerin dönmesine neden olacaktır!")
        msg.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

        if msg.exec_() == QtWidgets.QMessageBox.Yes:
            try:
                if isinstance(self.vehicle, OrangeCubeVehicle):
                    self.vehicle.armed = True
                    print("[SIM] Drone ARM edildi.")
                    QtWidgets.QMessageBox.information(self, "Başarılı", "Drone ARM edildi (Simülasyon).")
                else:
                    print("Arming motors...")
                    # Gerçek DroneKit arm komutu burada olacak
                    print("Motors Armed!")
                    QtWidgets.QMessageBox.information(self, "Başarılı", "Drone ARM edildi.")

                self.update_arm_disarm_buttons()
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Hata", f"Arm hatası: {e}")

    def disarm_drone(self):
        """Drone'u disarm (devre dışı bırakma) komutu gönderir."""
        if not self.vehicle:
            QtWidgets.QMessageBox.warning(self, "Hata", "Drone bağlı değil!")
            return

        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Question)
        msg.setText("Drone'u Disarm etmek istediğinize emin misiniz?")
        msg.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

        if msg.exec_() == QtWidgets.QMessageBox.Yes:
            try:
                if isinstance(self.vehicle, OrangeCubeVehicle):
                    self.vehicle.armed = False
                    print("[SIM] Drone DISARM edildi.")
                    QtWidgets.QMessageBox.information(self, "Başarılı", "Drone DISARM edildi (Simülasyon).")
                else:
                    print("Disarming motors...")
                    # Gerçek DroneKit disarm komutu burada olacak
                    print("Motors DISARMED!")
                    QtWidgets.QMessageBox.information(self, "Başarılı", "Drone DISARM edildi.")

                self.update_arm_disarm_buttons()
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Hata", f"Disarm hatası: {e}")

    def closeEvent(self, event):
        """Pencere kapanırken timer'ı durdur."""
        self.update_timer.stop()
        event.accept()

# Orange Cube Vehicle sınıfı
class OrangeCubeVehicle:
    def __init__(self, connection_string='tcp:127.0.0.1:5760', baud=57600):

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
        """Optimize edilmiş gerçek zamanlı veri okuma"""
        last_update = {}
        message_count = 0

        # Önemli mesajlar için çok kısa interval
        priority_messages = ['ATTITUDE', 'VFR_HUD', 'GPS_RAW_INT', 'SYS_STATUS']

        # Ultra önemli mesajlar (her seferinde işle)
        critical_messages = ['ATTITUDE', 'VFR_HUD']

        while self.running and self.is_connected:
            try:
                # TIMEOUT'u azalttık - daha hızlı mesaj alma
                msg = self.master.recv_match(blocking=True, timeout=0.05)  # 50ms

                if msg is None:
                    continue  # Timeout mesajını kaldırdık

                message_count += 1
                msg_type = msg.get_type()
                current_time = time.time()

                # Sadık her 500 mesajda bir yazdır (spam azaltma)
                if message_count % 500 == 0:
                    print(f"📊 {message_count} mesaj işlendi")

                # HIZLI GÜNCELLEME: Kritik mesajları her zaman işle
                if msg_type in critical_messages:
                    self._process_message(msg)
                    continue

                # Diğer mesajlar için interval kontrolü
                if msg_type in priority_messages:
                    min_interval = 0.02  # 20ms - çok hızlı
                else:
                    min_interval = 0.1  # 100ms - normal

                # Son güncelleme zamanı kontrolü
                if msg_type in last_update:
                    if current_time - last_update[msg_type] < min_interval:
                        continue

                last_update[msg_type] = current_time
                self._process_message(msg)

                # Debug mesajlarını azalttık
                if msg_type in ['HEARTBEAT'] and message_count % 100 == 0:
                    print(f"💓 Heartbeat - {message_count}")

            except Exception as e:
                print(f"❌ Hata: {e}")
                time.sleep(0.001)  # Çok kısa bekleme
                continue

    # Ayrıca web güncellemesini de hızlandırın:
    def update_web_data(self):
        """Web arayüzü için veri güncelleme - HIZLANDIRILDI"""
        while self.running:
            try:
                if self.is_connected:
                    # Veri hazırla
                    data = {
                        # Attitude (en önemli)
                        'roll': round(self.attitude.roll, 2),
                        'pitch': round(self.attitude.pitch, 2),
                        'yaw': round(self.attitude.yaw, 2),
                        'heading': round(self.heading, 1),

                        # GPS
                        'lat': round(self.location.global_frame.lat, 7),
                        'lon': round(self.location.global_frame.lon, 7),
                        'alt': round(self.location.global_relative_frame.alt, 2),
                        'gps_fix': self.gps_0.fix_type,
                        'satellites': self.gps_0.satellites_visible,

                        # Hız
                        'groundspeed': round(self.groundspeed, 1),
                        'climb_rate': round(self.climb_rate, 1) if hasattr(self, 'climb_rate') else 0,

                        # Sistem
                        'mode': self.mode.name,
                        'armed': self.armed,
                        'battery_voltage': round(self.battery.voltage, 2),
                        'battery_current': round(self.battery.current, 1),
                        'battery_level': self.battery.level,

                        'timestamp': time.time()
                    }

                    # WebSocket'e gönder (varsa)
                    if hasattr(self, 'websocket_clients'):
                        for client in self.websocket_clients:
                            try:
                                client.send(json.dumps(data))
                            except:
                                pass

                # HIZLI GÜNCELLEME: 50ms yerine 20ms
                time.sleep(0.02)  # 50Hz güncelleme hızı

            except Exception as e:
                print(f"Web güncelleme hatası: {e}")
                time.sleep(0.1)

    # Console çıktılarını da optimize edin:
    def print_status_optimized(self):
        """Optimize edilmiş durum yazdırma"""
        if self.is_connected:
            print(f"\r🚁 Mode: {self.mode.name} | "
                  f"GPS: {self.gps_0.fix_type}/{self.gps_0.satellites_visible} | "
                  f"Alt: {self.location.global_relative_frame.alt:.1f}m | "
                  f"Speed: {self.groundspeed:.1f}m/s | "
                  f"Bat: {self.battery.voltage:.1f}V | "
                  f"{'🔴ARMED' if self.armed else '🟢DISARMED'}", end='', flush=True)
        else:
            print("\r❌ Bağlantı yok", end='', flush=True)

    def _process_message(self, msg):
        """MAVLink mesajlarını işle"""
        with self.data_lock:
            try:
                if msg.get_type() == 'ATTITUDE':
                    # Mevcut attitude kodunuz
                    self.attitude.roll = math.degrees(msg.roll)
                    self.attitude.pitch = math.degrees(msg.pitch)
                    self.attitude.yaw = math.degrees(msg.yaw)
                    self.heading = math.degrees(msg.yaw) % 360

                    # Ek attitude bilgileri
                    self.attitude.rollspeed = math.degrees(msg.rollspeed)
                    self.attitude.pitchspeed = math.degrees(msg.pitchspeed)
                    self.attitude.yawspeed = math.degrees(msg.yawspeed)

                elif msg.get_type() == 'GPS_RAW_INT':
                    # Mevcut GPS kodunuz
                    self.gps_0.fix_type = msg.fix_type
                    self.gps_0.satellites_visible = msg.satellites_visible
                    self.location.global_frame.lat = msg.lat / 1e7
                    self.location.global_frame.lon = msg.lon / 1e7
                    self.location.global_relative_frame.alt = msg.alt / 1000.0

                    # Ek GPS bilgileri
                    self.gps_0.eph = msg.eph / 100.0  # GPS yatay doğruluk
                    self.gps_0.epv = msg.epv / 100.0  # GPS dikey doğruluk
                    self.gps_0.vel = msg.vel / 100.0  # GPS hız doğruluğu

                elif msg.get_type() == 'SYS_STATUS':
                    # Mevcut batarya kodunuz
                    self.battery.voltage = msg.voltage_battery / 1000.0
                    self.battery.current = msg.current_battery / 100.0
                    self.battery.level = msg.battery_remaining

                    # Ek sistem bilgileri
                    self.system_status = msg.onboard_control_sensors_health
                    self.cpu_load = msg.load / 10.0  # CPU yükü %

                elif msg.get_type() == 'HEARTBEAT':
                    # Mevcut heartbeat kodunuz
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.is_armable = not bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)

                    custom_mode = msg.custom_mode
                    mode_mapping = {
                        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
                        5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
                        13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE"
                    }
                    self.mode.name = mode_mapping.get(custom_mode, f"CUSTOM_{custom_mode}")

                elif msg.get_type() == 'VFR_HUD':
                    # Mevcut VFR kodunuz
                    self.groundspeed = msg.groundspeed

                    # Ek VFR bilgileri
                    self.airspeed = msg.airspeed
                    self.throttle = msg.throttle
                    self.climb_rate = msg.climb

                # YENİ MESAJ TİPLERİ - IHA BİLGİLERİ İÇİN
                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    # Detaylı konum bilgileri
                    self.location.global_frame.lat = msg.lat / 1e7
                    self.location.global_frame.lon = msg.lon / 1e7
                    self.location.global_frame.alt = msg.alt / 1000.0
                    self.location.global_relative_frame.alt = msg.relative_alt / 1000.0
                    self.velocity.x = msg.vx / 100.0
                    self.velocity.y = msg.vy / 100.0
                    self.velocity.z = msg.vz / 100.0
                    self.heading = msg.hdg / 100.0

                elif msg.get_type() == 'RC_CHANNELS':
                    # RC kumanda bilgileri
                    self.rc_channels = {
                        'roll': msg.chan1_raw,
                        'pitch': msg.chan2_raw,
                        'throttle': msg.chan3_raw,
                        'yaw': msg.chan4_raw,
                        'mode': msg.chan5_raw,
                        'aux1': msg.chan6_raw,
                        'aux2': msg.chan7_raw,
                        'aux3': msg.chan8_raw,
                    }

                elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                    # Motor/servo çıkışları
                    self.servo_outputs = {
                        'motor1': msg.servo1_raw,
                        'motor2': msg.servo2_raw,
                        'motor3': msg.servo3_raw,
                        'motor4': msg.servo4_raw,
                        'servo1': msg.servo5_raw,
                        'servo2': msg.servo6_raw,
                        'servo3': msg.servo7_raw,
                        'servo4': msg.servo8_raw,
                    }

                elif msg.get_type() == 'NAV_CONTROLLER_OUTPUT':
                    # Navigasyon kontrol bilgileri
                    self.nav_controller = {
                        'nav_roll': msg.nav_roll,
                        'nav_pitch': msg.nav_pitch,
                        'nav_bearing': msg.nav_bearing,
                        'target_bearing': msg.target_bearing,
                        'wp_dist': msg.wp_dist,
                        'alt_error': msg.alt_error,
                        'aspd_error': msg.aspd_error,
                        'xtrack_error': msg.xtrack_error,
                    }

                elif msg.get_type() == 'MISSION_CURRENT':
                    # Mevcut waypoint bilgisi
                    self.current_waypoint = msg.seq

                elif msg.get_type() == 'VIBRATION':
                    # Vibrasyon bilgileri
                    self.vibration = {
                        'x': msg.vibration_x,
                        'y': msg.vibration_y,
                        'z': msg.vibration_z,
                        'clipping_0': msg.clipping_0,
                        'clipping_1': msg.clipping_1,
                        'clipping_2': msg.clipping_2,
                    }

                elif msg.get_type() == 'SCALED_PRESSURE':
                    # Barometrik basınç
                    self.barometer = {
                        'pressure': msg.press_abs,  # hPa
                        'temperature': msg.temperature / 100.0,  # °C
                    }

                elif msg.get_type() == 'POWER_STATUS':
                    # Güç durumu
                    self.power_status = {
                        'vcc': msg.Vcc / 1000.0,  # V
                        'vservo': msg.Vservo / 1000.0,  # V
                        'flags': msg.flags,
                    }

            except Exception as e:
                print(f"Mesaj işleme hatası: {e}")

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



# --- DroneIconMacroElement Sınıfı ---
class DroneIconMacroElement(MacroElement):
    """Özel İHA ikonunu haritaya eklemek için özel sınıf"""

    def init(self):
        super(DroneIconMacroElement, self).init()
        self._template = Template("""
            {% macro script(this, kwargs) %}
                var droneIcon = L.divIcon({
                    className: 'drone-icon',
                    html: '<div style="font-size: 24px; color: red;"><i class="fa fa-fighter-jet"></i></div>',
                    iconSize: [30, 30],
                    iconAnchor: [15, 15]
                });

                var targetIcon = L.divIcon({
                    className: 'target-icon',
                    html: '<div style="font-size: 24px; color: green;"><i class="fa fa-crosshairs"></i></div>',
                    iconSize: [30, 30],
                    iconAnchor: [15, 15]
                });

                // İkonları değiştir
                {%- for marker in this._parent._children.values() %}
                    {% if marker.get_name() == 'drone_marker' %}
                        {{marker.get_name()}}.setIcon(droneIcon);
                    {% endif %}
                    {% if marker.get_name() == 'target_marker' %}
                        {{marker.get_name()}}.setIcon(targetIcon);
                    {% endif %}
                {%- endfor %}
            {% endmacro %}
            """)


# --- Ui_MainWindow Sınıfı ---
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1000, 700)  # Daha büyük pencere
        MainWindow.setWindowTitle("Kapsül Altay - İHA Kontrol Paneli")

        # Stil tanımlamaları
        self.buttonStyle = """
            QPushButton {
                background-color: #2c3e50;
                color: white;
                border-radius: 4px;
                padding: 6px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #34495e;
            }
            QPushButton:pressed {
                background-color: #1a2530;
            }
            QPushButton:disabled {
                background-color: #7f8c8d;
            }
        """

        self.panelStyle = """
            QGroupBox {
                border: 1px solid #bdc3c7;
                border-radius: 8px;
                background-color: #ecf0f1;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #2c3e50;
                font-weight: bold;
            }
        """

        self.labelStyle = """
            QLabel {
                font-size: 12px;
                color: #2c3e50;
                padding: 3px;
            }
        """

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.centralwidget.setStyleSheet("background-color: #f5f5f5;")

        # Ana layout
        self.mainLayout = QtWidgets.QHBoxLayout(self.centralwidget)

        # Sol panel layout
        self.leftPanel = QtWidgets.QVBoxLayout()
        self.leftPanel.setContentsMargins(10, 10, 10, 10)

        # Logo ve başlık
        self.logoLayout = QtWidgets.QHBoxLayout()
        self.logoLabel = QtWidgets.QLabel("KAPSÜL ALTAY")
        self.logoLabel.setStyleSheet("""
            font-size: 18px;
            font-weight: bold;
            color: #2c3e50;
            padding: 5px;
        """)
        self.logoLayout.addWidget(self.logoLabel)
        self.leftPanel.addLayout(self.logoLayout)

        # Bilgi Grubu
        self.infoGroup = QtWidgets.QGroupBox("İHA Bilgileri")
        self.infoGroup.setStyleSheet(self.panelStyle)
        self.infoLayout = QtWidgets.QVBoxLayout(self.infoGroup)

        # Bilgi etiketleri
        self.infoLabels = {}
        for label_name, label_text in [
            ("speed", "Hız: -- m/s"),
            ("altitude", "İrtifa: -- m"),
            ("latitude", "GPS Enlem: --"),
            ("longitude", "GPS Boylam: --"),
            ("time", "Uçuş Süresi: -- dk"),
            ("battery", "Batarya: --%"),
            ("status", "Durum: Bağlantı Bekleniyor")
        ]:
            label = QtWidgets.QLabel(label_text)
            label.setStyleSheet(self.labelStyle)
            self.infoLabels[label_name] = label
            self.infoLayout.addWidget(label)

        # Batarya göstergesi
        self.batteryLayout = QtWidgets.QHBoxLayout()
        self.batteryIcon = QtWidgets.QLabel()
        # Pil simgesi dosyasının varlığını kontrol et
        battery_icon_path = "battery_icon.png"
        if os.path.exists(battery_icon_path):
            self.batteryIcon.setPixmap(QtGui.QPixmap(battery_icon_path))
        self.batteryIcon.setFixedSize(20, 20)

        self.progressBar = QtWidgets.QProgressBar()
        self.progressBar.setValue(75)
        self.progressBar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                text-align: center;
                background-color: #ecf0f1;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                                stop:0 #e74c3c, stop:0.3 #f39c12, stop:1 #2ecc71);
            }
        """)

        self.batteryLayout.addWidget(self.progressBar)
        self.infoLayout.addLayout(self.batteryLayout)

        # Komut Grubu
        self.commandGroup = QtWidgets.QGroupBox("Komutlar")
        self.commandGroup.setStyleSheet(self.panelStyle)
        self.commandLayout = QtWidgets.QVBoxLayout(self.commandGroup)

        # Komut butonları
        self.buttons = {}
        button_data = [
            ("connect", "Bağlan", "connect.png"),
            ("disconnect", "Bağlantıyı Kes", "disconnect.png"),
            ("set_target", "Hedef Konum Belirle", "target.png"),
            ("track", "İzleme Modunu Aç", "track.png"),
            ("refresh_map", "Haritayı Yenile", "refresh.png"),
            ("take_photo", "Fotoğraf Çek", "camera.png"),
            ("return_home", "Ana Üsse Dön", "home.png"),
        ]

        for btn_name, btn_text, btn_icon_file in button_data:
            buttonLayout = QtWidgets.QHBoxLayout()
            button = QtWidgets.QPushButton(btn_text)
            button.setStyleSheet(self.buttonStyle)

            # İkon ekle (eğer dosya varsa)
            if os.path.exists(btn_icon_file):
                icon = QtGui.QIcon(btn_icon_file)
                button.setIcon(icon)

            self.buttons[btn_name] = button
            buttonLayout.addWidget(button)
            self.commandLayout.addLayout(buttonLayout)

            # Butonları fonksiyonlara bağla
            if btn_name == "refresh_map":
                button.clicked.connect(self.refreshMap)
            elif btn_name == "set_target":
                button.clicked.connect(self.setTargetLocation)
            elif btn_name == "connect":
                button.clicked.connect(self.connectDrone)
            elif btn_name == "disconnect":
                button.clicked.connect(self.disconnectDrone)
            elif btn_name == "track":
                button.clicked.connect(self.toggleTracking)
            elif btn_name == "take_photo":
                button.clicked.connect(self.takePhoto)
            elif btn_name == "return_home":
                button.clicked.connect(self.returnHome)

        # Başlangıçta bazı butonları devre dışı bırak
        self.buttons["disconnect"].setEnabled(False)
        self.buttons["set_target"].setEnabled(False)
        self.buttons["track"].setEnabled(False)
        self.buttons["take_photo"].setEnabled(False)
        self.buttons["return_home"].setEnabled(False)

        # Telemetri Grubu
        self.telemetryGroup = QtWidgets.QGroupBox("Telemetri")
        self.telemetryGroup.setStyleSheet(self.panelStyle)
        self.telemetryLayout = QtWidgets.QVBoxLayout(self.telemetryGroup)

        # Eğim, Yatış ve Yön Göstergeleri
        self.attitudeLabel = QtWidgets.QLabel("Eğim (Pitch): 0°")
        self.attitudeLabel.setStyleSheet(self.labelStyle)
        self.telemetryLayout.addWidget(self.attitudeLabel)

        self.rollLabel = QtWidgets.QLabel("Yatış (Roll): 0°")
        self.rollLabel.setStyleSheet(self.labelStyle)
        self.telemetryLayout.addWidget(self.rollLabel)

        self.headingLabel = QtWidgets.QLabel("Yön (Yaw): 0°")
        self.headingLabel.setStyleSheet(self.labelStyle)
        self.telemetryLayout.addWidget(self.headingLabel)

        # Titreşim ve Sıcaklık Göstergeleri
        self.vibrationLabel = QtWidgets.QLabel("Titreşim: 0.0 m/s²")
        self.vibrationLabel.setStyleSheet(self.labelStyle)
        self.telemetryLayout.addWidget(self.vibrationLabel)

        self.temperatureLabel = QtWidgets.QLabel("Sıcaklık: 25°C")
        self.temperatureLabel.setStyleSheet(self.labelStyle)
        self.telemetryLayout.addWidget(self.temperatureLabel)

        # Sol panele grupları ekle
        self.leftPanel.addWidget(self.infoGroup)
        self.leftPanel.addWidget(self.commandGroup)
        self.leftPanel.addWidget(self.telemetryGroup)
        self.leftPanel.addStretch()

        # Orta panel - harita
        self.mapGroup = QtWidgets.QGroupBox("Harita Görünümü")
        self.mapGroup.setStyleSheet(self.panelStyle)
        self.mapLayout = QtWidgets.QVBoxLayout(self.mapGroup)

        # Harita kontrolleri
        self.mapControlLayout = QtWidgets.QHBoxLayout()

        # Yakınlaştırma kontrolleri
        self.zoomInButton = QtWidgets.QPushButton("+")
        self.zoomInButton.setFixedSize(30, 30)
        self.zoomInButton.setStyleSheet(self.buttonStyle)
        self.zoomInButton.clicked.connect(self.zoomIn)

        self.zoomOutButton = QtWidgets.QPushButton("-")
        self.zoomOutButton.setFixedSize(30, 30)
        self.zoomOutButton.setStyleSheet(self.buttonStyle)
        self.zoomOutButton.clicked.connect(self.zoomOut)

        self.mapControlLayout.addWidget(self.zoomInButton)
        self.mapControlLayout.addWidget(self.zoomOutButton)
        self.mapControlLayout.addStretch()

        self.mapLayout.addLayout(self.mapControlLayout)

        # Harita webview
        self.webview = QWebEngineView()
        self.webview.setMinimumSize(600, 500)
        self.mapLayout.addWidget(self.webview)

        # Durum çubuğu
        self.statusBar = QtWidgets.QStatusBar()
        self.statusBar.showMessage("Kapsül Altay İHA Kontrol Paneli Hazır")

        # Layout'ları ana layout'a ekle
        self.mainLayout.addLayout(self.leftPanel, 1)
        self.mainLayout.addWidget(self.mapGroup, 3)

        MainWindow.setCentralWidget(self.centralwidget)
        MainWindow.setStatusBar(self.statusBar)

        # Menü bar oluştur
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.fileMenu = self.menuBar.addMenu("Uçuş Verileri")
        self.viewMenu = self.menuBar.addMenu("Görünüm")
        self.toolsMenu = self.menuBar.addMenu("Araçlar")
        self.helpMenu = self.menuBar.addMenu("Yardım")

        # Menü öğeleri ekle
        self.saveAction = QtWidgets.QAction("Uçuş Verilerini Kaydet", MainWindow)
        self.saveAction.setShortcut("Ctrl+S")
        self.fileMenu.addAction(self.saveAction)

        self.exitAction = QtWidgets.QAction("Çıkış", MainWindow)
        self.exitAction.setShortcut("Ctrl+Q")
        self.exitAction.triggered.connect(QtWidgets.qApp.quit)
        self.fileMenu.addAction(self.exitAction)

        MainWindow.setMenuBar(self.menuBar)

        # --- Eğim Göstergesi Penceresini Oluştur (Arm/Disarm butonlarıyla) ---
        self.attitudeWindow = AttitudeWindow()
        # Görünüm menüsüne eğim göstergesi eylemini ekle
        self.showAttitudeGaugeAction = QtWidgets.QAction("Eğim Göstergesi", MainWindow)
        self.showAttitudeGaugeAction.triggered.connect(self.showAttitudeGauge)
        self.viewMenu.addAction(self.showAttitudeGaugeAction)


        self.current_location = [39.925533, 32.866287]  # Ankara
        self.target_location = [39.935533, 32.876287]
        self.home_location = [39.925533, 32.866287]
        self.current_zoom = 14
        self.tracking_enabled = False
        self.initMap()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateDroneData)
        self.timer.start(100)

        self.flight_time = 0
        self.flight_timer = QtCore.QTimer()
        self.flight_timer.timeout.connect(self.updateFlightTime)

        self.is_connected = False
        self.is_flying = False # Başlangıçta uçuşta değil
        self.vehicle = None # vehicle objesini başlangıçta None olarak ayarla

    def showAttitudeGauge(self):
        """Eğim göstergesi penceresini gösterir ve drone objesini aktarır."""
        # Drone objesini AttitudeWindow'a gönder
        self.attitudeWindow.set_vehicle(self.vehicle)
        self.attitudeWindow.show()
        # self.attitudeWindow.activateWindow()
        # self.attitudeWindow.raise_()

    # Güncellenmiş connectDrone fonksiyonu
    def connectDrone(self):
        """Orange Cube'a bağlan"""
        try:
            print("Orange Cube bağlantısı deneniyor...")

            # --- GERÇEK ORANGE CUBE BAĞLANTISI ---
            # COM portunu kendi sisteminize göre ayarlayın (COM3, COM9, /dev/ttyUSB0 vs.)
            self.vehicle = OrangeCubeVehicle(connection_string='tcp:127.0.0.1:5762')



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
            print(f"Battery: {self.vehicle.battery.level}% ({self.vehicle.battery.voltage}V)")
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
            self.buttons["set_target"].setEnabled(True)
            self.buttons["track"].setEnabled(True)
            self.buttons["take_photo"].setEnabled(True)
            self.buttons["return_home"].setEnabled(True)

            # Veri güncelleme timer'ını başlat
            self.timer.start(100)  # 10Hz güncelleme

            # AttitudeWindow'u güncelle
            if hasattr(self, 'attitudeWindow'):
                self.attitudeWindow.set_vehicle(self.vehicle)
                self.attitudeWindow.update_arm_disarm_buttons()

        except Exception as e:
            print(f"❌ Orange Cube bağlantı hatası: {e}")
            self.statusBar.showMessage(f"Bağlantı hatası: {e}")
            self.is_connected = False

    def initMap(self):
        """Haritayı başlat"""
        m = folium.Map(
            location=self.current_location,
            zoom_start=self.current_zoom,
            tiles="OpenStreetMap"
        )

        drone_marker = folium.Marker(
            location=self.current_location,
            popup="Kapsül Altay",
            icon=folium.Icon(color="red", icon="plane", prefix="fa"),
            name="drone_marker"
        ).add_to(m)

        folium.Marker(
            location=self.home_location,
            popup="Ana Üs",
            icon=folium.Icon(color="blue", icon="home", prefix="fa")
        ).add_to(m)

        target_marker = folium.Marker(
            location=self.target_location,
            popup="Hedef Konum",
            icon=folium.Icon(color="green", icon="flag", prefix="fa"),
            name="target_marker"
        ).add_to(m)

        folium.PolyLine(
            locations=[self.current_location, self.target_location],
            color="blue",
            weight=2,
            opacity=0.7
        ).add_to(m)

        folium.Circle(
            location=self.current_location,
            radius=500,
            color="red",
            fill=True,
            fill_opacity=0.1
        ).add_to(m)

        drone_icon_element = DroneIconMacroElement()
        m.get_root().add_child(drone_icon_element)

        map_path = "kapsul_altay_map.html"
        m.save(map_path)
        self.webview.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(map_path)))
        self.statusBar.showMessage(f"Harita yüklendi")

    def refreshMap(self):
        """Haritayı yenile"""
        self.initMap()
        self.statusBar.showMessage("Harita yenilendi")

    def zoomIn(self):
        """Haritayı yakınlaştır"""
        if self.current_zoom < 18:
            self.current_zoom += 1
            self.initMap()

    def zoomOut(self):
        """Haritayı uzaklaştır"""
        if self.current_zoom > 3:
            self.current_zoom -= 1
            self.initMap()

    def setTargetLocation(self):
        """Hedef konumu ayarla"""
        self.target_location = [
            self.current_location[0] + random.uniform(-0.005, 0.005),
            self.current_location[1] + random.uniform(-0.005, 0.005)
        ]
        self.initMap()
        self.statusBar.showMessage(f"Yeni hedef konum: {self.target_location[0]:.6f}, {self.target_location[1]:.6f}")

        if self.is_connected and self.tracking_enabled and self.vehicle:
            try:
                # from dronekit import LocationGlobalRelative # Eğer kullanılacaksa buraya veya en üste eklenmeli
                target_alt = self.vehicle.location.global_relative_frame.alt if hasattr(self.vehicle.location, 'global_relative_frame') and self.vehicle.location.global_relative_frame.alt is not None else 10
                # self.vehicle.simple_goto(LocationGlobalRelative(self.target_location[0], self.target_location[1], target_alt))
                print(f"Hedefe gitme komutu gönderildi: {self.target_location}")
            except Exception as e:
                print(f"Hedefe gitme komutu hatası: {e}")
                self.statusBar.showMessage(f"Hedefe gitme komutu hatası: {e}")

    def updateDroneData(self):
        """IHA verilerini güncelle"""
        # Eğer bağlı değilsek veya vehicle objesi yoksa güncelleme yapma
        if not self.is_connected or not hasattr(self, 'vehicle') or self.vehicle is None:
            # Sadece Attitude penceresi açıksa simülasyon verisi gösterelim
            if hasattr(self, 'attitudeWindow') and self.attitudeWindow.isVisible():
                # Bağlı değilken veya vehicle None ise simülasyon verileri üret
                sim_pitch = random.uniform(-25, 25)  # -25 ile +25 derece arası
                sim_roll = random.uniform(-30, 30)  # -30 ile +30 derece arası
                sim_yaw = random.uniform(0, 360)  # 0 ile 360 derece arası
                self.attitudeWindow.update_gauge(sim_pitch, sim_roll, sim_yaw)
            return

        v = self.vehicle

        try:
            # Data lock kullanarak thread-safe veri erişimi
            with v.data_lock:
                # HIZ BİLGİSİ - VFR_HUD ve GLOBAL_POSITION_INT'den al
                speed = 0
                if hasattr(v, 'groundspeed') and v.groundspeed is not None:
                    speed = v.groundspeed
                elif hasattr(v, 'velocity'):
                    # velocity objesi varsa x,y,z bileşenlerini kontrol et
                    vx = getattr(v.velocity, 'x', 0) or 0
                    vy = getattr(v.velocity, 'y', 0) or 0
                    speed = math.sqrt(vx ** 2 + vy ** 2)

                # İRTİFA BİLGİSİ - GLOBAL_POSITION_INT'den relative altitude'u al
                altitude = 0
                if hasattr(v, 'location'):
                    if hasattr(v.location, 'global_relative_frame'):
                        altitude = getattr(v.location.global_relative_frame, 'alt', 0) or 0
                    elif hasattr(v.location, 'global_frame'):
                        altitude = getattr(v.location.global_frame, 'alt', 0) or 0

                # GPS BİLGİSİ - GLOBAL_POSITION_INT ve GPS_RAW_INT'den al
                lat = 0
                lon = 0
                gps_sats = 0
                gps_fix = 0

                if hasattr(v, 'location') and hasattr(v.location, 'global_frame'):
                    lat = getattr(v.location.global_frame, 'lat', 0) or 0
                    lon = getattr(v.location.global_frame, 'lon', 0) or 0

                # GPS detay bilgileri
                if hasattr(v, 'gps_0'):
                    gps_sats = getattr(v.gps_0, 'satellites_visible', 0) or 0
                    gps_fix = getattr(v.gps_0, 'fix_type', 0) or 0

                # BATARYA BİLGİSİ
                battery_level = 0
                battery_voltage = 0
                battery_current = 0

                if hasattr(v, 'battery'):
                    battery_level = getattr(v.battery, 'level', 0) or 0
                    battery_voltage = getattr(v.battery, 'voltage', 0) or 0
                    battery_current = getattr(v.battery, 'current', 0) or 0

                # ATTITUDE BİLGİSİ - Zaten derece cinsinden kaydediliyor
                pitch = 0
                roll = 0
                yaw = 0

                if hasattr(v, 'attitude'):
                    pitch = getattr(v.attitude, 'pitch', 0) or 0
                    roll = getattr(v.attitude, 'roll', 0) or 0
                    yaw = getattr(v.attitude, 'yaw', 0) or 0

                # HEADING BİLGİSİ
                heading = getattr(v, 'heading', 0) or 0

                # UÇUŞ MODU VE ARM DURUMU
                flight_mode = "Bilinmiyor"
                armed_status = False

                if hasattr(v, 'mode') and hasattr(v.mode, 'name'):
                    flight_mode = getattr(v.mode, 'name', 'Bilinmiyor') or 'Bilinmiyor'

                armed_status = getattr(v, 'armed', False) or False

            # Mevcut konumu güncelle
            self.current_location = [lat, lon]

            # UI ELEMANLARINI GÜNCELLE
            # IHA Bilgileri bölümü - Sol taraftaki bilgiler
            if hasattr(self, 'infoLabels'):
                # Hız bilgisi - m/s ve km/h olarak göster
                speed_kmh = speed * 3.6
                self.infoLabels["speed"].setText(f"Hız: {speed:.1f} m/s ({speed_kmh:.1f} km/h)")

                # İrtifa bilgisi
                self.infoLabels["altitude"].setText(f"İrtifa: {altitude:.1f} m")

                # GPS koordinatları
                self.infoLabels["latitude"].setText(f"GPS Enlem: {lat:.6f}")
                self.infoLabels["longitude"].setText(f"GPS Boylam: {lon:.6f}")

                # Batarya durumu
                self.infoLabels["battery"].setText(f"Batarya: %{battery_level} ({battery_voltage:.1f}V)")

                # Sistem durumu
                arm_text = "Armlı" if armed_status else "Disarmlı"
                self.infoLabels["status"].setText(f"Durum: {flight_mode} ({arm_text})")

            # Progress bar güncelleme
            if hasattr(self, 'progressBar'):
                self.progressBar.setValue(int(battery_level))

            # Telemetri bölümü - Sağ taraftaki detaylı bilgiler
            if hasattr(self, 'attitudeLabel'):
                self.attitudeLabel.setText(f"Eğim (Pitch): {pitch:.1f}°")
            if hasattr(self, 'rollLabel'):
                self.rollLabel.setText(f"Yatış (Roll): {roll:.1f}°")
            if hasattr(self, 'headingLabel'):
                self.headingLabel.setText(f"Yön (Yaw): {yaw:.1f}° / Heading: {heading:.1f}°")

            # GPS detay bilgileri
            if hasattr(self, 'gpsLabel'):
                gps_fix_names = {0: "Yok", 1: "GPS Yok", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
                fix_name = gps_fix_names.get(gps_fix, f"Bilinmiyor({gps_fix})")
                self.gpsLabel.setText(f"GPS: {gps_sats} uydu, Fix: {fix_name}")

            # Hız detayları
            if hasattr(self, 'speedLabel'):
                airspeed = getattr(v, 'airspeed', 0) or 0
                climb_rate = getattr(v, 'climb_rate', 0) or 0
                self.speedLabel.setText(
                    f"Yer Hızı: {speed:.1f} m/s, Hava Hızı: {airspeed:.1f} m/s, Tırmanma: {climb_rate:.1f} m/s")

            # Batarya detayları
            if hasattr(self, 'batteryDetailLabel'):
                self.batteryDetailLabel.setText(
                    f"Batarya: {battery_voltage:.2f}V, {battery_current:.2f}A, %{battery_level}")

            # Sıcaklık bilgisi
            temperature = 25  # Varsayılan
            if hasattr(v, 'barometer') and isinstance(v.barometer, dict):
                temperature = v.barometer.get('temperature', 25)
            if hasattr(self, 'temperatureLabel'):
                self.temperatureLabel.setText(f"Sıcaklık: {temperature:.1f}°C")

            # Vibrasyon bilgisi
            if hasattr(self, 'vibrationLabel'):
                if hasattr(v, 'vibration') and isinstance(v.vibration, dict):
                    vib_x = v.vibration.get('x', 0)
                    vib_y = v.vibration.get('y', 0)
                    vib_z = v.vibration.get('z', 0)
                    vib_total = math.sqrt(vib_x ** 2 + vib_y ** 2 + vib_z ** 2)
                    self.vibrationLabel.setText(f"Titreşim: {vib_total:.2f} m/s²")
                else:
                    self.vibrationLabel.setText("Titreşim: Bilinmiyor")

            # Attitude penceresi güncelle
            if hasattr(self, 'attitudeWindow') and self.attitudeWindow.isVisible():
                self.attitudeWindow.update_gauge(pitch, roll, yaw)
                if hasattr(self.attitudeWindow, 'update_arm_disarm_buttons'):
                    self.attitudeWindow.update_arm_disarm_buttons()

            # Debug için konsola yazdır
            if speed > 0 or altitude > 0:
                print(f"DEBUG - Hız: {speed:.1f} m/s, İrtifa: {altitude:.1f} m, GPS: {lat:.6f}, {lon:.6f}")

        except Exception as e:
            if hasattr(self, 'statusBar'):
                self.statusBar.showMessage(f"Veri güncelleme hatası: {e}")
            print(f"Veri güncelleme hatası: {e}")
            import traceback
            traceback.print_exc()

    def updateFlightTime(self):
        """Uçuş süresini güncelle"""
        # Uçuş süresi, drone bağlı ve armlıysa saysın
        if self.is_connected and self.vehicle and getattr(self.vehicle, 'armed', False):
            self.flight_time += 1
            minutes = self.flight_time // 60
            seconds = self.flight_time % 60
            self.infoLabels["time"].setText(f"Uçuş Süresi: {minutes:02d}:{seconds:02d}")
        else:
            self.flight_time = 0 # Disarm ise süreyi sıfırla
            self.infoLabels["time"].setText(f"Uçuş Süresi: 00:00")

    # Bağlantıyı kesme fonksiyonu da güncellenmeli
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
            self.buttons["set_target"].setEnabled(False)
            self.buttons["track"].setEnabled(False)
            self.buttons["take_photo"].setEnabled(False)
            self.buttons["return_home"].setEnabled(False)

            print("✅ Orange Cube bağlantısı kesildi")

        except Exception as e:
            print(f"❌ Bağlantı kesme hatası: {e}")

    def toggleTracking(self):
        """İzleme modunu aç/kapat"""
        self.tracking_enabled = not self.tracking_enabled
        if self.tracking_enabled:
            self.statusBar.showMessage("İzleme modu aktif - Hedefe yönleniyor")
            self.buttons["track"].setText("İzleme Modunu Kapat")
            # Gerçek drone'da burada simple_goto komutu gönderilir
            # if self.is_connected and self.vehicle:
            #     try:
            #         from dronekit import LocationGlobalRelative # Eğer kullanılacaksa buraya veya en üste eklenmeli
            #         target_alt = self.vehicle.location.global_relative_frame.alt if hasattr(self.vehicle.location, 'global_relative_frame') and self.vehicle.location.global_relative_frame.alt is not None else 10
            #         self.vehicle.simple_goto(LocationGlobalRelative(self.target_location[0], self.target_location[1], target_alt))
            #     except Exception as e:
            #         print(f"Hedefe gitme komutu hatası: {e}")
            #         self.statusBar.showMessage(f"Hedefe gitme komutu hatası: {e}")

        else:
            self.statusBar.showMessage("İzleme modu devre dışı")
            self.buttons["track"].setText("İzleme Modunu Aç")

    def takePhoto(self):
        """Fotoğraf çek"""
        self.statusBar.showMessage("Fotoğraf çekildi")

        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Information)
        msg.setText("Fotoğraf başarıyla çekildi")
        msg.setWindowTitle("Kapsül Altay")
        msg.exec_()

    def returnHome(self):
        """Ana üsse dön"""
        self.target_location = self.home_location.copy()
        self.tracking_enabled = True
        self.buttons["track"].setText("İzleme Modunu Kapat")
        self.statusBar.showMessage("Ana üsse dönüş başlatıldı")
        self.initMap()
        # Gerçek drone'da burada RTL (Return To Launch) modu etkinleştirilir
        # if self.is_connected and self.vehicle:
        #     try:
        #         from dronekit import VehicleMode # Eğer kullanılacaksa buraya veya en üste eklenmeli
        #         self.vehicle.mode = VehicleMode("RTL")
        #     except Exception as e:
        #         print(f"RTL komutu hatası: {e}")
        #         self.statusBar.showMessage(f"RTL komutu hatası: {e}")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
