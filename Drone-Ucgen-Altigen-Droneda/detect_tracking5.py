import cv2
import numpy as np
import time

def apply_digital_zoom(frame, zoom_factor=1.7):
    """Kırmızı kare algılama için digital zoom uygular"""
    h, w = frame.shape[:2]
    
    # Zoom sonrası boyutlar
    new_h, new_w = int(h / zoom_factor), int(w / zoom_factor)
    
    # Merkezi bul
    start_x = (w - new_w) // 2
    start_y = (h - new_h) // 2
    
    # Görüntüyü kırp
    cropped = frame[start_y:start_y + new_h, start_x:start_x + new_w]
    
    # Orijinal boyuta yeniden boyutlandır
    zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
    
    return zoomed

def detect_target_and_get_output(frame, model=None, search_for=None):
    # Görüntüyü kopyala
    vis = frame.copy()
    
    if search_for == "square":
        frame_to_process = apply_digital_zoom(frame, zoom_factor=1.8)
        vis = frame_to_process.copy()  # Zoom'lu görüntüyü göster
    else:
        frame_to_process = frame
        vis = frame.copy()
    
    # Görüntü işleme adımları
    frame_blurred = cv2.GaussianBlur(frame_to_process, (3, 3), 0)
    hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)

    # DÜZELTME 1: Renk aralıklarını ayarladık
    # Kırmızı renk için daha geniş HSV aralıkları
    lower_red1 = np.array([0, 80, 80])      # Saturation ve Value değerlerini azalttık
    upper_red1 = np.array([10, 255, 255])   
    lower_red2 = np.array([170, 80, 80])    # Saturation ve Value değerlerini azalttık
    upper_red2 = np.array([180, 255, 255])  

    # Mavi renk için de aynı şekilde
    lower_blue = np.array([100, 80, 80])    # Saturation ve Value değerlerini azalttık
    upper_blue = np.array([130, 255, 255])  
    
    # Maskeleme
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask_red1, mask_red2)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Morfolojik işlemler
    kernel_small = np.ones((3,3), np.uint8)
    kernel_medium = np.ones((5,5), np.uint8)
    
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel_small, iterations=1)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_medium, iterations=1)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel_small, iterations=1)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel_medium, iterations=1)
    
    # Gürültü azaltma
    red_mask = cv2.medianBlur(red_mask, 3)
    blue_mask = cv2.medianBlur(blue_mask, 3)
    
    # En büyük alanı bulmak için
    detected_target = None
    detected_center = None
    largest_area = 0
    
    # KIRMIZI ŞEKİLLER
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > 800:
            perimeter = cv2.arcLength(contour, True)
            if perimeter < 120:
                continue
            
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                vertices = len(approx)
                shape_name = ""
                shape_type = None
                
                # ÜÇGEN: detect_tracking3.py'den (EN İYİ)
                if vertices == 3:
                    area_perimeter_ratio = area / (perimeter * perimeter)
                    if 0.015 < area_perimeter_ratio < 0.12:
                        shape_name = "KIRMIZI UCGEN"
                        shape_type = "triangle"
                        
                # KARE: detect_tracking4.py'den (EN İYİ)        
                elif vertices == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.75 <= aspect_ratio <= 1.25:
                        rect_area = w * h
                        extent = float(area) / rect_area
                        if extent > 0.5:
                            shape_name = "KIRMIZI KARE"
                            shape_type = "square"
                            
                
                # search_for filtresi uygula
                if shape_type and (search_for is None or 
                    (search_for == "triangle" and shape_type == "triangle") or
                    (search_for == "square" and shape_type == "square")):
                    
                    # En büyük alanı bul
                    if area > largest_area:
                        largest_area = area
                        detected_target = shape_type
                        detected_center = (cx, cy)
                    
                    # Görselleştirme - Kırmızı renge çiz
                    cv2.drawContours(vis, [contour], -1, (0, 255, 0), 4)
                    cv2.putText(vis, shape_name, (cx-80, cy-30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)

    # MAVİ ŞEKİLLER: detect_tracking4.py'den (EN İYİ)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in blue_contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            perimeter = cv2.arcLength(contour, True)
            if perimeter < 100:
                continue
            
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                vertices = len(approx)
                
                # ALTIGEN: detect_tracking4.py'den (EN İYİ)
                if vertices == 6:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.75 <= aspect_ratio <= 1.25:  # Kare toleransı
                        rect_area = w * h
                        extent = float(area) / rect_area
                        if extent > 0.5:  # Kare toleransı
                            shape_name = "MAVI ALTIGEN"
                            shape_type = "hexagon"
                            
                            # search_for filtresi uygula
                            if search_for is None or search_for == "hexagon":
                                
                                # En büyük alanı bul
                                if area > largest_area:
                                    largest_area = area
                                    detected_target = shape_type
                                    detected_center = (cx, cy)
                                
                                # Görselleştirme
                                cv2.drawContours(vis, [contour], -1, (0, 255, 0), 4)
                                cv2.putText(vis, shape_name, (cx-80, cy-30),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)

    # Durum bilgisi ekle
    status_text = f"Aranan: {search_for if search_for else 'Tumu'}"
    cv2.putText(vis, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    if detected_target:
        result_text = f"Tespit: {detected_target.upper()}"
        cv2.putText(vis, result_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return vis, detected_target, detected_center


# Test fonksiyonu
def test_detection():
    """Test fonksiyonu - birleştirilmiş fonksiyonu test eder"""
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Kamera açılamadı!")
        return
    
    print("Test modu - q ile çıkış")
    print("EN İYİ algılama: Kırmızı üçgen (tracking3) + Kırmızı kare + Mavi altıgen (tracking4)")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        vis, target_type, center = detect_target_and_get_output(frame)
        
        cv2.imshow('Birlestirilmis En Iyi Algilama', vis)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Eğer dosya direkt çalıştırılırsa test fonksiyonunu çalıştır
    test_detection()
