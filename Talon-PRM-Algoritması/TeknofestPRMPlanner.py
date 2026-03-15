#!/usr/bin/env python3
"""
Teknofest Savaşan İHA Yarışması - PRM Path Planning Package
Otonom uçuş projesi için optimize edilmiş paket
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import random
import math
import heapq
from matplotlib.patches import Circle
import os
import logging

# Loglama ayarları
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

try:
    import pyproj

    PYPROJ_AVAILABLE = True
except ImportError:
    PYPROJ_AVAILABLE = False
    logger.warning("pyproj kütüphanesi bulunamadı. Basit koordinat dönüşümü kullanılacak.")


class TeknofesPRMPlanner:
    """Teknofest için optimize edilmiş PRM yol planlayıcısı"""

    def __init__(self, safety_margin=15.0, num_nodes=1000, connection_radius=150.0):
        """
        Args:
            safety_margin: HSS bölgelerinden güvenlik mesafesi (metre)
            num_nodes: Üretilecek rastgele düğüm sayısı
            connection_radius: Düğümler arası maksimum bağlantı mesafesi (metre)
        """
        self.safety_margin = safety_margin
        self.num_nodes = num_nodes
        self.connection_radius = connection_radius

        # İç değişkenler
        self.hss_zones = []
        self.start_gps = None
        self.goal_gps = None
        self.forbidden_zones = []
        self.start_point = None
        self.goal_point = None
        self.nodes = []
        self.edges = []
        self.graph = {}
        self.transformer = None

    def _setup_projection(self):
        """UTM projeksiyonu kurulumu"""
        if not PYPROJ_AVAILABLE:
            logger.info("pyproj kullanılamıyor, basit dönüşüm kullanılacak")
            return

        try:
            # Ortalama boylam hesapla
            avg_lon = np.mean([zone["hssBoylam"] for zone in self.hss_zones] +
                              [self.start_gps["lon"], self.goal_gps["lon"]])

            # UTM zone hesapla
            utm_zone = int((avg_lon + 180) / 6) + 1

            # Koordinat dönüşüm objesi
            self.transformer = pyproj.Transformer.from_crs(
                "EPSG:4326",  # WGS84 (GPS)
                f"EPSG:326{utm_zone}",  # UTM Zone N
                always_xy=True
            )
            logger.info(f"UTM Zone {utm_zone} kullanılıyor")

        except Exception as e:
            logger.warning(f"UTM projeksiyonu kurulamadı: {e}")
            self.transformer = None

    def _gps_to_meters(self, lat, lon):
        """GPS koordinatlarını metre cinsine çevirir"""
        if self.transformer:
            try:
                x, y = self.transformer.transform(lon, lat)
                return (x, y)
            except Exception as e:
                logger.warning(f"UTM dönüşümü başarısız: {e}")

        # Fallback: Basit hesaplama
        return self._simple_gps_to_meters(lat, lon)

    def _simple_gps_to_meters(self, lat, lon):
        """Basit GPS-metre dönüşümü (yaklaşık)"""
        # Referans noktası (ilk HSS bölgesinin merkezi)
        ref_lat = self.hss_zones[0]["hssEnlem"]
        ref_lon = self.hss_zones[0]["hssBoylam"]

        # Yaklaşık dönüşüm (Türkiye enlemleri için)
        lat_to_m = 111320  # 1 derece enlem ≈ 111.32 km
        lon_to_m = 111320 * math.cos(math.radians(ref_lat))

        x = (lon - ref_lon) * lon_to_m
        y = (lat - ref_lat) * lat_to_m

        return (x, y)

    def _meters_to_gps(self, x, y):
        """Metre koordinatlarını GPS'e çevirir"""
        if self.transformer:
            try:
                lon, lat = self.transformer.transform(x, y, direction='INVERSE')
                return lat, lon
            except Exception as e:
                logger.warning(f"GPS dönüşümü başarısız: {e}")

        # Fallback: Basit hesaplama
        return self._simple_meters_to_gps(x, y)

    def _simple_meters_to_gps(self, x, y):
        """Basit metre-GPS dönüşümü"""
        ref_lat = self.hss_zones[0]["hssEnlem"]
        ref_lon = self.hss_zones[0]["hssBoylam"]

        lat_to_m = 111320
        lon_to_m = 111320 * math.cos(math.radians(ref_lat))

        lat = ref_lat + (y / lat_to_m)
        lon = ref_lon + (x / lon_to_m)

        return lat, lon

    def _convert_hss_to_meters(self):
        """HSS bölgelerini metre cinsine çevirir"""
        forbidden_zones = []
        for zone in self.hss_zones:
            x, y = self._gps_to_meters(zone["hssEnlem"], zone["hssBoylam"])
            radius = zone["hssYaricap"]
            forbidden_zones.append((x, y, radius))
        return forbidden_zones

    def _calculate_workspace(self):
        """Çalışma alanını hesaplar"""
        all_x = [zone[0] for zone in self.forbidden_zones] + [self.start_point[0], self.goal_point[0]]
        all_y = [zone[1] for zone in self.forbidden_zones] + [self.start_point[1], self.goal_point[1]]

        padding = 200  # 200 metre padding
        self.min_x = min(all_x) - padding
        self.max_x = max(all_x) + padding
        self.min_y = min(all_y) - padding
        self.max_y = max(all_y) + padding

    def _is_point_valid(self, x, y):
        """Noktanın HSS bölgelerinde olup olmadığını kontrol eder"""
        for zone_x, zone_y, radius in self.forbidden_zones:
            distance = math.sqrt((x - zone_x) ** 2 + (y - zone_y) ** 2)
            if distance < (radius + self.safety_margin):
                return False
        return True

    def _is_path_valid(self, x1, y1, x2, y2, step_size=5.0):
        """İki nokta arasındaki yolun geçerli olup olmadığını kontrol eder"""
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        num_steps = int(distance / step_size) + 1

        for i in range(num_steps + 1):
            t = i / num_steps if num_steps > 0 else 0
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            if not self._is_point_valid(x, y):
                return False
        return True

    def _generate_random_nodes(self):
        """Rastgele geçerli düğümler üretir"""
        self.nodes = [self.start_point, self.goal_point]

        attempts = 0
        max_attempts = self.num_nodes * 15

        while len(self.nodes) < self.num_nodes + 2 and attempts < max_attempts:
            x = random.uniform(self.min_x, self.max_x)
            y = random.uniform(self.min_y, self.max_y)

            if self._is_point_valid(x, y):
                self.nodes.append((x, y))

            attempts += 1

        logger.info(f"{len(self.nodes)} geçerli düğüm üretildi")

    def _connect_nodes(self):
        """Düğümleri birbirine bağlar"""
        self.graph = {i: [] for i in range(len(self.nodes))}
        self.edges = []

        connection_count = 0

        for i in range(len(self.nodes)):
            for j in range(i + 1, len(self.nodes)):
                x1, y1 = self.nodes[i]
                x2, y2 = self.nodes[j]

                distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

                if distance <= self.connection_radius:
                    if self._is_path_valid(x1, y1, x2, y2):
                        self.graph[i].append((j, distance))
                        self.graph[j].append((i, distance))
                        self.edges.append((i, j))
                        connection_count += 1

        logger.info(f"{connection_count} bağlantı oluşturuldu")

    def _dijkstra(self, start_idx=0, goal_idx=1):
        """Dijkstra algoritması ile en kısa yolu bulur"""
        distances = {i: float('inf') for i in range(len(self.nodes))}
        distances[start_idx] = 0
        previous = {i: None for i in range(len(self.nodes))}
        visited = set()

        heap = [(0, start_idx)]

        while heap:
            current_distance, current_node = heapq.heappop(heap)

            if current_node in visited:
                continue

            visited.add(current_node)

            if current_node == goal_idx:
                break

            for neighbor, weight in self.graph[current_node]:
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(heap, (distance, neighbor))

        # Yolu yeniden oluştur
        path = []
        current = goal_idx
        while current is not None:
            path.append(current)
            current = previous[current]

        return path[::-1] if path and path[-1] == start_idx else None

    def _path_to_gps(self, path):
        """Yolu GPS koordinatlarına çevirir"""
        if not path:
            return None

        gps_path = []
        for x, y in path:
            lat, lon = self._meters_to_gps(x, y)
            gps_path.append({"lat": lat, "lon": lon})

        return gps_path

    def _calculate_path_statistics(self, path):
        """Yol istatistiklerini hesaplar"""
        if not path or len(path) < 2:
            return {"total_distance": 0, "segment_distances": []}

        total_distance = 0
        segment_distances = []

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            total_distance += dist
            segment_distances.append(dist)

        return {
            "total_distance": total_distance,
            "segment_distances": segment_distances,
            "waypoint_count": len(path)
        }

    def plan_path(self, hss_data, start_gps, goal_gps):
        """
        Ana yol planlama fonksiyonu

        Args:
            hss_data: JSON verisi veya dosya yolu
            start_gps: {"lat": float, "lon": float} başlangıç GPS koordinatı
            goal_gps: {"lat": float, "lon": float} hedef GPS koordinatı

        Returns:
            dict: {
                "success": bool,
                "gps_waypoints": [{"lat": float, "lon": float}, ...],
                "total_distance": float,
                "waypoint_count": int,
                "message": str
            }
        """
        try:
            logger.info("🚁 Teknofest PRM yol planlama başlıyor...")

            # 1. HSS verilerini yükle
            if isinstance(hss_data, str):
                # Dosya yolu verilmiş
                with open(hss_data, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            else:
                # Direkt JSON verisi
                data = hss_data

            self.hss_zones = data["hss_koordinat_bilgileri"]
            self.start_gps = start_gps
            self.goal_gps = goal_gps

            logger.info(f"📍 {len(self.hss_zones)} HSS bölgesi yüklendi")

            # 2. Koordinat dönüşümü
            self._setup_projection()

            # 3. GPS'ten metre sistemine çevir
            self.forbidden_zones = self._convert_hss_to_meters()
            self.start_point = self._gps_to_meters(start_gps["lat"], start_gps["lon"])
            self.goal_point = self._gps_to_meters(goal_gps["lat"], goal_gps["lon"])

            # 4. Çalışma alanını hesapla
            self._calculate_workspace()

            # 5. PRM algoritması
            self._generate_random_nodes()
            self._connect_nodes()

            # 6. En kısa yolu bul
            logger.info("🎯 En kısa yol hesaplanıyor...")
            path_indices = self._dijkstra()

            if path_indices is None:
                logger.error("❌ Yol bulunamadı!")
                return {
                    "success": False,
                    "gps_waypoints": None,
                    "total_distance": 0,
                    "waypoint_count": 0,
                    "message": "Yol bulunamadı! Parametreleri kontrol edin."
                }

            # 7. Koordinatları çevir ve istatistikleri hesapla
            path_coordinates = [self.nodes[i] for i in path_indices]
            gps_path = self._path_to_gps(path_coordinates)
            stats = self._calculate_path_statistics(path_coordinates)

            logger.info(f"✅ Yol bulundu! {stats['waypoint_count']} waypoint, {stats['total_distance']:.1f}m")

            return {
                "success": True,
                "gps_waypoints": gps_path,
                "total_distance": round(stats['total_distance'], 1),
                "waypoint_count": stats['waypoint_count'],
                "segment_distances": [round(d, 1) for d in stats['segment_distances']],
                "message": "Yol planlama başarılı!"
            }

        except Exception as e:
            logger.error(f"❌ Yol planlama hatası: {e}")
            return {
                "success": False,
                "gps_waypoints": None,
                "total_distance": 0,
                "waypoint_count": 0,
                "message": f"Hata: {str(e)}"
            }

    def visualize_path(self, result_data, save_plot=False, show_plot=True):
        """
        Yol planlama sonucunu görselleştirir

        Args:
            result_data: plan_path() fonksiyonundan dönen sonuç
            save_plot: Grafiği dosyaya kaydet
            show_plot: Grafiği ekranda göster
        """
        if not result_data["success"]:
            logger.error("Görselleştirme için geçerli yol bulunamadı!")
            return None

        try:
            # Matplotlib backend'ini ayarla
            if show_plot:
                try:
                    import matplotlib
                    # Backend'i zorla ayarla
                    current_backend = matplotlib.get_backend()
                    logger.info(f"Mevcut backend: {current_backend}")

                    if 'inline' in current_backend.lower() or 'agg' in current_backend.lower():
                        matplotlib.use('TkAgg')
                        logger.info("Backend TkAgg'ye değiştirildi")
                except Exception as e:
                    logger.warning(f"Backend değiştirilemedi: {e}")

            # Figure oluştur
            plt.close('all')  # Önceki figürleri kapat
            fig, ax = plt.subplots(figsize=(14, 10))

            # HSS bölgelerini çiz
            for i, zone in enumerate(self.hss_zones):
                zone_x, zone_y, radius = self.forbidden_zones[i]

                # Güvenlik marjı
                safety_circle = Circle((zone_x, zone_y), radius + self.safety_margin,
                                       color='red', alpha=0.2,
                                       label='HSS + Güvenlik' if i == 0 else "")
                ax.add_patch(safety_circle)

                # HSS bölgesi
                hss_circle = Circle((zone_x, zone_y), radius,
                                    color='darkred', alpha=0.7,
                                    label='HSS Bölgesi' if i == 0 else "")
                ax.add_patch(hss_circle)

                # HSS ID
                ax.text(zone_x, zone_y, f'HSS-{zone["id"]}',
                        ha='center', va='center', fontweight='bold',
                        color='white', fontsize=9)

            # PRM düğümleri
            if self.nodes:
                nodes_x, nodes_y = zip(*self.nodes)
                ax.scatter(nodes_x, nodes_y, c='lightblue', s=6, alpha=0.5,
                           label='PRM Düğümleri')

            # Kenarlar
            for edge in self.edges:
                x1, y1 = self.nodes[edge[0]]
                x2, y2 = self.nodes[edge[1]]
                ax.plot([x1, x2], [y1, y2], 'gray', alpha=0.2, linewidth=0.3)

            # Planlanan yol
            gps_waypoints = result_data["gps_waypoints"]
            path_coords = []
            for wp in gps_waypoints:
                x, y = self._gps_to_meters(wp["lat"], wp["lon"])
                path_coords.append((x, y))

            if path_coords:
                path_x, path_y = zip(*path_coords)
                ax.plot(path_x, path_y, 'green', linewidth=4, label='Planlanan Yol')
                ax.scatter(path_x, path_y, c='green', s=80, zorder=5, edgecolors='white')

                # Waypoint numaraları
                for i, (x, y) in enumerate(path_coords):
                    ax.annotate(f'WP{i}', (x, y), xytext=(6, 6),
                                textcoords='offset points', fontsize=9,
                                fontweight='bold', color='darkgreen')

            # Start ve Goal
            ax.scatter(self.start_point[0], self.start_point[1], c='blue', s=250,
                       marker='*', label='Başlangıç', zorder=10, edgecolors='white')
            ax.scatter(self.goal_point[0], self.goal_point[1], c='orange', s=250,
                       marker='*', label='Hedef', zorder=10, edgecolors='white')

            # Ayarlar
            ax.set_xlim(self.min_x, self.max_x)
            ax.set_ylim(self.min_y, self.max_y)
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right')
            ax.set_title(f'Teknofest PRM - Mesafe: {result_data["total_distance"]}m',
                         fontsize=14, fontweight='bold')
            ax.set_xlabel('X Koordinatı (metre)')
            ax.set_ylabel('Y Koordinatı (metre)')
            ax.set_aspect('equal')

            plt.tight_layout()

            # Kaydetme
            if save_plot:
                plt.savefig('teknofest_path_plan.png', dpi=300, bbox_inches='tight')
                logger.info("📊 Grafik kaydedildi: teknofest_path_plan.png")

            # Gösterme - Kalıcı yöntemler
            if show_plot:
                logger.info("📊 Grafik gösteriliyor... Kapatmak için X'e tıklayın")

                # Birden fazla yöntem dene
                try:
                    # Yöntem 1: Interactive mode ile
                    plt.ion()
                    plt.show()
                    plt.draw()

                    # Pencereyi canlı tut
                    manager = fig.canvas.manager
                    manager.window.wm_deiconify()
                    manager.window.lift()

                    # Konsol mesajı
                    print("\n" + "=" * 50)
                    print("🎨 GRAFIK PENCERESI AÇILDI!")
                    print("📊 Rotayı görmek için pencereyi kontrol edin")
                    print("❌ Kapatmak için grafik penceresindeki X'e tıklayın")
                    print("⌨️  Veya buraya 'q' yazıp Enter'a basın")
                    print("=" * 50)

                    # Kullanıcı girdisi bekle
                    while True:
                        try:
                            user_input = input("\nGrafiği kapatmak için 'q' + Enter: ").strip().lower()
                            if user_input == 'q':
                                break
                        except KeyboardInterrupt:
                            break
                        except:
                            break

                    plt.close(fig)

                except Exception as e:
                    logger.warning(f"Interactive gösterim başarısız: {e}")

                    # Yöntem 2: Basit gösterim
                    try:
                        plt.ioff()
                        plt.show(block=True)
                    except Exception as e2:
                        logger.error(f"Grafik gösterilemedi: {e2}")
                        logger.info(
                            "💡 GUI problemi olabilir. Grafik 'teknofest_path_plan.png' dosyasına kaydediliyor...")
                        if not save_plot:
                            plt.savefig('teknofest_path_plan.png', dpi=300, bbox_inches='tight')
                            logger.info("📁 Dosya kaydedildi: teknofest_path_plan.png")

            return fig, ax

        except Exception as e:
            logger.error(f"Görselleştirme hatası: {e}")
            return None


# Kolay kullanım fonksiyonları
def plan_teknofest_path(hss_data, start_gps, goal_gps,
                        safety_margin=15.0, num_nodes=1000, connection_radius=150.0,
                        visualize=False, save_plot=False):
    """
    Teknofest için basit yol planlama fonksiyonu

    Args:
        hss_data: JSON verisi veya dosya yolu
        start_gps: {"lat": float, "lon": float}
        goal_gps: {"lat": float, "lon": float}
        safety_margin: HSS'lerden güvenlik mesafesi (metre)
        num_nodes: PRM düğüm sayısı
        connection_radius: Düğüm bağlantı yarıçapı (metre)
        visualize: Görselleştirme yap
        save_plot: Grafiği dosyaya kaydet

    Returns:
        dict: Yol planlama sonucu
    """
    planner = TeknofesPRMPlanner(
        safety_margin=safety_margin,
        num_nodes=num_nodes,
        connection_radius=connection_radius
    )

    result = planner.plan_path(hss_data, start_gps, goal_gps)

    # Görselleştirme istenmişse
    if visualize and result["success"]:
        planner.visualize_path(result, save_plot=save_plot, show_plot=True)

    return result


def force_show_plot():
    """Matplotlib penceresini zorla göster"""
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        plt.ion()
        plt.show(block=False)
        plt.pause(0.1)
        return True
    except Exception as e:
        logger.warning(f"Pencere açılamadı: {e}")
        return False


def show_path_simple(result_data, planner, create_waypoints=False, waypoint_filename="mission_auto.waypoints"):
    """
    Basit ve güvenilir görselleştirme fonksiyonu

    Args:
        result_data: plan_path() sonucu
        planner: TeknofesPRMPlanner objesi
        create_waypoints: .waypoints dosyası oluşturulsun mu
        waypoint_filename: Waypoints dosya adı (varsayılan: mission_auto.waypoints)
    """
    if not result_data["success"]:
        print("❌ Görselleştirilecek yol bulunamadı!")
        return

    try:
        # Backend'i zorla ayarla
        import matplotlib
        matplotlib.use('TkAgg')

        # Yeni figür oluştur
        plt.figure(figsize=(12, 8))
        plt.clf()  # Temizle

        # HSS bölgeleri
        for i, zone in enumerate(planner.hss_zones):
            zone_x, zone_y, radius = planner.forbidden_zones[i]

            circle = plt.Circle((zone_x, zone_y), radius + planner.safety_margin,
                                color='red', alpha=0.3)
            plt.gca().add_patch(circle)

            circle2 = plt.Circle((zone_x, zone_y), radius, color='darkred', alpha=0.7)
            plt.gca().add_patch(circle2)

            plt.text(zone_x, zone_y, f'HSS-{zone["id"]}',
                     ha='center', va='center', color='white', fontweight='bold')

        # Yol
        gps_waypoints = result_data["gps_waypoints"]
        path_x, path_y = [], []
        for wp in gps_waypoints:
            x, y = planner._gps_to_meters(wp["lat"], wp["lon"])
            path_x.append(x)
            path_y.append(y)

        # Yolu çiz
        plt.plot(path_x, path_y, 'g-', linewidth=3, label='Planlanan Yol')
        plt.scatter(path_x, path_y, c='green', s=60, zorder=5)

        # Start ve Goal
        plt.scatter(planner.start_point[0], planner.start_point[1],
                    c='blue', s=200, marker='*', label='Başlangıç', zorder=10)
        plt.scatter(planner.goal_point[0], planner.goal_point[1],
                    c='orange', s=200, marker='*', label='Hedef', zorder=10)

        plt.xlim(planner.min_x, planner.max_x)
        plt.ylim(planner.min_y, planner.max_y)
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.title(f'Teknofest PRM Yol Planı - {result_data["total_distance"]}m')
        plt.xlabel('X (metre)')
        plt.ylabel('Y (metre)')
        plt.axis('equal')

        # Göster ve bekle
        print("\n🎨 Grafik penceresi açılıyor...")
        print("❌ Kapatmak için penceredeki X'e tıklayın")

        plt.tight_layout()
        plt.show(block=True)

        # Eğer waypoint dosyası isteniyorsa oluştur
        if create_waypoints:
            with open(waypoint_filename, 'w') as f:
                f.write("QGC WPL 110\n")
                for i, wp in enumerate(gps_waypoints):
                    lat = wp["lat"]
                    lon = wp["lon"]
                    alt = 20  # Öntanımlı yükseklik, gerekirse parametre yapılabilir
                    current_wp = 1 if i == 0 else 0
                    autocontinue = 1
                    f.write(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{lat:.7f}\t{lon:.7f}\t{alt}\t{autocontinue}\n")

            print(f"✅ .waypoints dosyası oluşturuldu: {waypoint_filename}")

    except Exception as e:
        print(f"❌ Görselleştirme hatası: {e}")
        print("💾 Grafik dosyaya kaydediliyor...")
        plt.savefig('emergency_path_plot.png', dpi=200, bbox_inches='tight')
        print("📁 Dosya kaydedildi: emergency_path_plot.png")


def interactive_path_viewer(result_data, planner, filename="mission.waypoints"):
    """
    Etkileşimli yol görüntüleyici - Thread kullanarak
    """
    try:
        import threading
        import time

        def show_plot():
            try:
                import matplotlib
                matplotlib.use('TkAgg')

                fig, ax = plt.subplots(figsize=(12, 8))

                # HSS bölgeleri
                for i, zone in enumerate(planner.hss_zones):
                    zone_x, zone_y, radius = planner.forbidden_zones[i]
                    circle = Circle((zone_x, zone_y), radius + planner.safety_margin,
                                    color='red', alpha=0.3)
                    ax.add_patch(circle)
                    circle2 = Circle((zone_x, zone_y), radius, color='darkred', alpha=0.7)
                    ax.add_patch(circle2)
                    ax.text(zone_x, zone_y, f'HSS-{zone["id"]}',
                            ha='center', va='center', color='white', fontweight='bold')

                # Yol
                gps_waypoints = result_data["gps_waypoints"]
                path_coords = []
                for wp in gps_waypoints:
                    x, y = planner._gps_to_meters(wp["lat"], wp["lon"])
                    path_coords.append((x, y))

                if path_coords:
                    path_x, path_y = zip(*path_coords)
                    ax.plot(path_x, path_y, 'green', linewidth=3, label='Yol')
                    ax.scatter(path_x, path_y, c='green', s=60, zorder=5)

                # Start ve Goal
                ax.scatter(planner.start_point[0], planner.start_point[1],
                           c='blue', s=200, marker='*', label='Başlangıç')
                ax.scatter(planner.goal_point[0], planner.goal_point[1],
                           c='orange', s=200, marker='*', label='Hedef')

                ax.set_xlim(planner.min_x, planner.max_x)
                ax.set_ylim(planner.min_y, planner.max_y)
                ax.grid(True, alpha=0.3)
                ax.legend()
                ax.set_title(f'Teknofest PRM - {result_data["total_distance"]}m')
                ax.set_aspect('equal')

                plt.tight_layout()
                plt.show()

            except Exception as e:
                print(f"Thread görselleştirme hatası: {e}")

        # Thread başlat
        plot_thread = threading.Thread(target=show_plot)
        plot_thread.daemon = True
        plot_thread.start()

        print("🎨 Grafik thread'i başlatıldı...")
        print("⏱️  Pencere açılması bekleniyor...")

        # Thread'in bitmesini bekle
        plot_thread.join(timeout=10)

        if plot_thread.is_alive():
            print("⚠️ Grafik gösterimi zaman aşımına uğradı")

    except Exception as e:
        print(f"Thread hatası: {e}")
        print("🔄 Basit görselleştirme deneniyor...")
        show_path_simple(result_data, planner)
    """Yol planını dosyaya kaydet (görsel göstermeden)"""
    if not result_data["success"]:
        logger.error("Kaydedilecek geçerli yol bulunamadı!")
        return False

    try:
        import matplotlib
        matplotlib.use('Agg')  # GUI olmayan backend

        fig, ax = plt.subplots(figsize=(14, 10))

        # HSS bölgelerini çiz
        for i, zone in enumerate(planner.hss_zones):
            zone_x, zone_y, radius = planner.forbidden_zones[i]

            safety_circle = Circle((zone_x, zone_y), radius + planner.safety_margin,
                                   color='red', alpha=0.2)
            ax.add_patch(safety_circle)

            hss_circle = Circle((zone_x, zone_y), radius,
                                color='darkred', alpha=0.7)
            ax.add_patch(hss_circle)

            ax.text(zone_x, zone_y, f'HSS-{zone["id"]}',
                    ha='center', va='center', fontweight='bold',
                    color='white', fontsize=9)

        # Yol
        gps_waypoints = result_data["gps_waypoints"]
        path_coords = []
        for wp in gps_waypoints:
            x, y = planner._gps_to_meters(wp["lat"], wp["lon"])
            path_coords.append((x, y))

        if path_coords:
            path_x, path_y = zip(*path_coords)
            ax.plot(path_x, path_y, 'green', linewidth=4, label='Planlanan Yol')
            ax.scatter(path_x, path_y, c='green', s=80, zorder=5)

        # Start ve Goal
        ax.scatter(planner.start_point[0], planner.start_point[1], c='blue', s=250,
                   marker='*', label='Başlangıç', zorder=10)
        ax.scatter(planner.goal_point[0], planner.goal_point[1], c='orange', s=250,
                   marker='*', label='Hedef', zorder=10)

        ax.set_xlim(planner.min_x, planner.max_x)
        ax.set_ylim(planner.min_y, planner.max_y)
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title(f'Teknofest PRM - Mesafe: {result_data["total_distance"]}m')

        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
        logger.info(f"📊 Grafik kaydedildi: {filename}")
        return True

    except Exception as e:
        logger.error(f"Kaydetme hatası: {e}")
        return False


def create_test_scenario():
    """Test senaryosu oluşturur"""
    test_data = {
        "hss_koordinat_bilgileri": [
            {"id": 0, "hssEnlem": 40.23260922, "hssBoylam": 29.00573015, "hssYaricap": 50},
            {"id": 1, "hssEnlem": 40.23351019, "hssBoylam": 28.99976492, "hssYaricap": 50},
            {"id": 2, "hssEnlem": 40.23105297, "hssBoylam": 29.00744677, "hssYaricap": 75},
            {"id": 3, "hssEnlem": 40.23090554, "hssBoylam": 29.00221109, "hssYaricap": 120}
        ]
    }

    start_gps = {"lat": 40.23000000, "lon": 28.99500000}
    goal_gps = {"lat": 40.23500000, "lon": 29.01000000}

    return test_data, start_gps, goal_gps


# Test fonksiyonu
if __name__ == "__main__":
    # Test senaryosunu çalıştır
    logger.info("🧪 Test senaryosu çalıştırılıyor...")

    test_data, start_gps, goal_gps = create_test_scenario()

    # Yol planla
    result = plan_teknofest_path(
        hss_data=test_data,
        start_gps=start_gps,
        goal_gps=goal_gps,
        safety_margin=20.0
    )

    if result["success"]:
        logger.info(f"✅ Test başarılı! Mesafe: {result['total_distance']}m")
        logger.info(f"📍 Waypoint sayısı: {result['waypoint_count']}")

        # Görselleştir
        planner = TeknofesPRMPlanner(safety_margin=20.0)
        planner.plan_path(test_data, start_gps, goal_gps)
        planner.visualize_path(result, show_plot=True)
    else:
        logger.error(f"❌ Test başarısız: {result['message']}")