import cv2
import numpy as np
from ultralytics import YOLO
import argparse
import torch
import time


def identify_polygon(mask, min_side_length=15):
    """
    Identify the number of sides in a polygon from a binary mask
    Back to basics approach focused on robust triangle detection

    Args:
        mask: Binary mask from YOLO segmentation
        min_side_length: Minimum side length to consider (to filter noise)

    Returns:
        num_sides: Number of sides detected
        approx_polygon: Approximated polygon points
        is_quadrilateral: Boolean indicating if shape is a true quadrilateral with balanced sides
    """
    # Find contours in the mask
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return 0, None, False

    # Get the largest contour
    contour = max(contours, key=cv2.contourArea)

    # Get perimeter of the contour
    perimeter = cv2.arcLength(contour, True)

    # Approximate the polygon - use a fixed epsilon
    # More consistent across object sizes
    epsilon = 0.03 * perimeter  # 3% of perimeter for approximation
    approx_polygon = cv2.approxPolyDP(contour, epsilon, True)

    # Count number of sides (vertices)
    num_sides = len(approx_polygon)

    # Initialize is_quadrilateral flag
    is_quadrilateral = False

    # Calculate side lengths for shape analysis
    sides = []
    if num_sides > 2:  # Need at least 3 points to form a polygon
        for i in range(num_sides):
            pt1 = approx_polygon[i][0]
            pt2 = approx_polygon[(i + 1) % num_sides][0]
            side_length = np.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
            if side_length >= min_side_length:
                sides.append(side_length)

    # Special detection for shapes with 4 detected sides
    if num_sides == 4:
        if sides:
            # Calculate the balance ratio between sides
            max_side = max(sides)
            min_side = min(sides)

            # Check if all sides are within 30% of each other
            if min_side > 0 and (max_side / min_side) <= 1.3:
                is_quadrilateral = True
            else:
                # If sides are too imbalanced, interpret as triangle
                num_sides = 3

    # Filter small sides to avoid noise for other shapes
    if num_sides > 3 and not is_quadrilateral:
        filtered_sides = []
        for i in range(num_sides):
            pt1 = approx_polygon[i][0]
            pt2 = approx_polygon[(i + 1) % num_sides][0]
            side_length = np.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
            if side_length >= min_side_length:
                filtered_sides.append((pt1, pt2))

        num_sides = len(filtered_sides)

    # Simple regularization - objects with 2 sides don't exist in reality
    if num_sides < 3:
        num_sides = 3  # Assume at least a triangle

    return num_sides, approx_polygon, is_quadrilateral


def determine_color(frame, mask):
    """
    Simplified and robust color detection function
    Uses direct HSV thresholds for more reliable red and blue detection

    Args:
        frame: Original RGB frame
        mask: Binary mask to apply

    Returns:
        color_name: String describing the color ("red", "blue", "other")
        avg_color: Average HSV color values in the masked region
    """
    # Apply mask to frame
    masked = cv2.bitwise_and(frame, frame, mask=mask.astype(np.uint8))

    # Convert to HSV for better color analysis
    hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)

    # Get non-zero pixels (where mask is applied)
    non_zero = hsv[mask > 0]

    if len(non_zero) == 0:
        return "unknown", (0, 0, 0)

    # Create red and blue masks using direct HSV thresholds
    # Red is at both ends of hue spectrum (0-10 and 170-180)
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # Blue is around 100-130 in OpenCV's HSV
    lower_blue = np.array([100, 70, 50])
    upper_blue = np.array([130, 255, 255])

    # Apply color masks to the masked region
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Count pixels of each color
    red_pixels = cv2.countNonZero(mask_red)
    blue_pixels = cv2.countNonZero(mask_blue)
    total_pixels = cv2.countNonZero(mask)

    # Calculate percentages
    red_percent = red_pixels / total_pixels if total_pixels > 0 else 0
    blue_percent = blue_pixels / total_pixels if total_pixels > 0 else 0

    # Simple threshold-based decision
    # Require at least 20% of pixels to match a color for reliability
    min_percent = 0.2

    # For debugging
    avg_h = np.mean(non_zero[:, 0]) if len(non_zero) > 0 else 0
    avg_s = np.mean(non_zero[:, 1]) if len(non_zero) > 0 else 0
    avg_v = np.mean(non_zero[:, 2]) if len(non_zero) > 0 else 0

    # Decision logic
    if red_percent > min_percent and red_percent > blue_percent:
        return "red", (avg_h, avg_s, avg_v)
    elif blue_percent > min_percent and blue_percent > red_percent:
        return "blue", (avg_h, avg_s, avg_v)
    else:
        # As a fallback, use a simpler hue-based check
        if len(non_zero) > 0:
            avg_h = np.mean(non_zero[:, 0])
            if (avg_h < 15 or avg_h > 165) and avg_s > 50:  # Red hues with decent saturation
                return "red", (avg_h, avg_s, avg_v)
            elif 100 < avg_h < 140 and avg_s > 50:  # Blue hues with decent saturation
                return "blue", (avg_h, avg_s, avg_v)

        return "other", (avg_h, avg_s, avg_v)


def process_single_frame_optimized(frame, model,search_for=None, size=(640, 640)):
    """
    Process a single frame using YOLO model with segmentation,
    showing only final detection for performance optimization

    Improved with more reliable color detection and shape recognition
    Added target zone and drop functionality

    Args:
        frame: Input frame from video
        model: Loaded YOLO segmentation model
        size: Desired processing size (width, height)

    Returns:
        Processed frame with only final detection and target zone
    """
    # Resize frame to target size for consistent processing
    orig_h, orig_w = frame.shape[:2]
    resized_frame = cv2.resize(frame, size)

    # Run detection with segmentation
    results = model(resized_frame, verbose=False)

    # Get the first result (assuming batch size 1)
    result = results[0]

    # Create a copy of the original frame for drawing
    annotated_frame = frame.copy()

    # Track if we found target objects
    found_objects = []

    # Define screen center coordinates
    center_x_screen = orig_w // 2
    center_y_screen = orig_h // 2

    # Define center circle parameters
    center_circle_radius = 100

    # Draw center circle (target zone)
    cv2.circle(annotated_frame, (center_x_screen, center_y_screen),
               center_circle_radius, (0, 255, 255), 2)  # Yellow circle


    # Flag to track if object is inside target zone
    is_inside = False

    # Process each detection with mask
    if hasattr(result, 'masks') and result.masks is not None:
        for i, mask in enumerate(result.masks):
            # Get the class ID and name
            cls_id = int(result.boxes[i].cls[0])
            cls_name = result.names[cls_id]

            # Get confidence score
            conf = float(result.boxes[i].conf[0])

            # Get segmentation mask
            seg_mask = mask.data.cpu().numpy()[0]  # Get first mask

            # Resize mask to original frame size
            seg_mask = cv2.resize(seg_mask, (size[0], size[1]))
            seg_mask = cv2.resize(seg_mask, (orig_w, orig_h))

            # Convert to binary mask
            binary_mask = (seg_mask > 0.5).astype(np.uint8)

            # Determine color first
            color_name, avg_color = determine_color(frame, binary_mask)

            # Identify polygon properties
            num_sides, approx_polygon, is_quadrilateral = identify_polygon(binary_mask)

            # Toleranslı hedef tespiti kodları
            sides_tolerance = 1  # Kenar sayısında +/- tolerans

            # Üçgen toleransı (2-4 kenar ve kırmızı, ancak quadrilateral kontrolü ile)
            is_triangle = (abs(num_sides - 3) <= sides_tolerance and color_name == "red" and not is_quadrilateral)

            # Dörtgen kontrolü (quadrilateral olarak tespit edilmiş ve dörtgen şekli ise)
            is_square = (is_quadrilateral and color_name == "red")

            # Altıgen toleransı (5-7 kenar ve mavi)
            is_hexagon = abs(num_sides - 6) <= sides_tolerance and color_name == "blue"

            # Debugging information
            if conf > 0.7:  # Only log higher confidence detections
                (f"Detection: Color={color_name}, Sides={num_sides}, Is_Quad={is_quadrilateral}, Conf={conf:.2f}")
                (f"HSV Avg: H={avg_color[0]:.1f}, S={avg_color[1]:.1f}, V={avg_color[2]:.1f}")
                # Calculate area
                area = np.sum(binary_mask)
                (f"Area: {area} pixels")

            # Eğer toleranslı hedef tespiti yapılırsa
            if ((search_for == "triangle" and is_triangle) or 
                (search_for == "hexagon" and is_hexagon) or
                (search_for == "square" and is_square)):
                # Şekli belirle
                if is_triangle:
                    shape_name = "Triangle"
                    shape_sides = 3
                elif is_square:
                    shape_name = "Square"
                    shape_sides = 4
                else:  # is_hexagon
                    shape_name = "Hexagon"
                    shape_sides = 6

                # Hedef nesneyi kaydet
                found_objects.append(f"{color_name.capitalize()} {shape_name}")

                # Çokgeni çiz
                if approx_polygon is not None:
                    if color_name == "red":
                        polygon_color = (0, 0, 255)  # Red in BGR
                    else:  # Blue
                        polygon_color = (255, 0, 0)  # Blue in BGR

                    cv2.polylines(annotated_frame, [approx_polygon], isClosed=True,
                                  color=polygon_color, thickness=2)

                    # Merkez noktayı hesapla
                    M = cv2.moments(approx_polygon)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])

                        # Merkez noktayı işaretle
                        cv2.drawMarker(annotated_frame, (center_x, center_y),
                                       color=polygon_color, markerType=cv2.MARKER_CROSS,
                                       markerSize=20, thickness=2)

                        # Etiket ekle (toleranslı olduğunu belirten bilgi de dahil)
                        detected_info = f"(Actual: {num_sides}, Target: {shape_sides}"
                        if is_quadrilateral:
                            detected_info += ", Balanced sides"
                        detected_info += f") {conf:.2f}"

                        # Sadece kırmızı üçgen veya mavi altıgen için çizgi çiz
                        if (is_triangle and color_name == "red") or (is_hexagon and color_name == "blue"):
                            # Hesapla: Merkez noktalar arası mesafe
                            distance = np.sqrt((center_x - center_x_screen) ** 2 + (center_y - center_y_screen) ** 2)

                            # Tespit edilmiş şeklin merkezi, hedef bölgenin içinde mi?
                            is_inside = distance <= center_circle_radius

                            # Çizgi rengi: yeşil (normal) veya kırmızı (hedef bölge içinde)
                            line_color = (0, 0, 255) if is_inside else (0, 255, 0)  # Kırmızı veya Yeşil

                            # Şeklin merkezinden ekranın merkezine çizgi çiz
                            cv2.line(annotated_frame, (center_x, center_y),
                                     (center_x_screen, center_y_screen), line_color, 2)

                            # Mesafe bilgisini göster
                            dist_text = f"Distance: {distance:.1f}px"
                            cv2.putText(annotated_frame, dist_text,
                                        (center_x + 30, center_y),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, line_color, 2)

            # Additionally visualize color detection regardless of shape match
            # This helps debug color detection issues
            else:
                # If shape doesn't match but detection confidence is decent
                if conf > 0.6:
                    # Just draw the contour for visual feedback
                    if color_name == "red":
                        color_vis = (0, 0, 255)  # Red in BGR
                        shape_txt = "Red (no shape match)"
                    elif color_name == "blue":
                        color_vis = (255, 0, 0)  # Blue in BGR
                        shape_txt = "Blue (no shape match)"
                    else:
                        color_vis = (0, 255, 0)  # Green for other
                        shape_txt = "Other color"

    return annotated_frame

def process_video_optimized(video_path, model_path, start_second=0):
    """
    Process a video using YOLO segmentation to identify red triangles and blue hexagons
    with performance optimization - only showing Final Detection and not saving output

    Args:
        video_path: Path to input video
        model_path: Path to YOLO segmentation model
        start_second: Time to start processing (in seconds)
    """
    # Check if CUDA is available and set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    (f"Using device: {device}")

    # Force CUDA if available
    if torch.cuda.is_available():
        torch.cuda.set_device(0)  # Use first GPU
        (f"CUDA device selected: {torch.cuda.get_device_name(0)}")
        (f"Available CUDA devices: {torch.cuda.device_count()}")
    else:
        ("WARNING: CUDA is not available, using CPU. Performance may be slow.")

    # Load model and send to appropriate device
    (f"Loading YOLO segmentation model from {model_path}...")
    model = YOLO(model_path)
    model.to(device)  # Move model to CUDA device if available

    # Open video
    (f"Opening video from {video_path}...")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        ("Error: Could not open video.")
        return

    # Get video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    (f"Video properties: {width}x{height}, {fps} FPS, {total_frames} frames")
    (f"Processing at 640x640 resolution for YOLO model input")
    (f"Display window will be at 1920x1080 resolution")

    # Create a resizable window with the specified resolution
    cv2.namedWindow("Final Detection (Press ESC to exit)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Final Detection (Press ESC to exit)", 1920, 1080)

    # Set starting position
    start_frame = int(start_second * fps)
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    (f"Processing video from {start_second} seconds...")

    frame_count = 0
    total_process_frames = total_frames - start_frame  # No limit, process until end

    # FPS hesaplama için değişkenler
    prev_time = 0
    curr_time = 0
    fps_display = 0

    try:
        # Create progress bar
        from tqdm import tqdm
        progress_bar = tqdm(total=total_process_frames, desc="Processing frames")

        while True:
            # FPS hesaplama
            curr_time = time.time()
            if curr_time - prev_time > 0:  # Sıfıra bölmeyi önle
                fps_display = 1.0 / (curr_time - prev_time)
            prev_time = curr_time

            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1
            current_time = (start_frame + frame_count) / fps

            # Update progress bar
            progress_bar.update(1)
            progress_bar.set_description(
                f"Processing frame {frame_count}/{total_process_frames} (Time: {current_time:.2f}s)")

            # Process frame with optimized function - only final detection
            result_frame = process_single_frame_optimized(frame, model)

            # Display frame (window already resized to 1920x1080)
            cv2.imshow("Final Detection (Press ESC to exit)", result_frame)

            # Check for exit
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                ("\nProcessing cancelled by user.")
                break

        progress_bar.close()

    except KeyboardInterrupt:
        ("\nProcessing interrupted by user.")

    finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        ("\nProcessing complete.")

def detect_target_and_get_output(frame, model, search_for):
    """Bir karede görev gereği aranan hedefi (kırmızı üçgen veya mavi altıgen)
    tespit eder, anotasyonlu görüntüyü ve hedef merkezini döndürür.

    Args:
        frame (np.ndarray): BGR görüntü.
        model (ultralytics.YOLO): YOLO segmentasyon modeli.
        search_for (str): "triangle" veya "hexagon" – hangisi aranacak?

    Returns:
        annotated (np.ndarray): Anotasyonlu görüntü (debug amaçlı).
        target_type (str | None): "triangle", "hexagon" ya da None.
        center (tuple[int, int] | None): (x, y) piksel koordinatı.
    """
    # Önce optimizasyonlu tek-kare işleme (arkadaşının fonksiyonu)
    annotated = process_single_frame_optimized(frame, model,search_for)

    target_type = None
    center = None

    # YOLO sonuçları
    results = model(frame, verbose=False)
    result = results[0]

    if not hasattr(result, "masks") or result.masks is None:
        return annotated, target_type, center

    # Kenar toleransı
    tol = 1

    for i, mask in enumerate(result.masks):
        # Segmentasyon maskesini tam boyuta çıkar
        seg = mask.data.cpu().numpy()[0]
        seg = cv2.resize(seg, (frame.shape[1], frame.shape[0]))
        bin_mask = (seg > 0.5).astype(np.uint8)

        # Renk & şekil analizleri
        color, _ = determine_color(frame, bin_mask)
        sides, approx, is_quad = identify_polygon(bin_mask)

        is_triangle = abs(sides - 3) <= tol and color == "red" and not is_quad
        is_hexagon = abs(sides - 6) <= tol and color == "blue"
        is_square = is_quad and color == "red"

        # Görevde hangi hedef aranıyor?
        if search_for == "triangle" and is_triangle:
            target_type = "triangle"
        elif search_for == "hexagon" and is_hexagon:
            target_type = "hexagon"
        elif search_for == "square" and is_square:  # Bu satırı ekleyin
    	    target_type = "square"
        else:
            continue  # Bu maske hedef değil, sıradakine bak

        # Hedef bulundu – merkez hesapla
        if approx is not None:
            M = cv2.moments(approx)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                center = (cx, cy)
        break  # İlk uygun hedef yeterli

    return annotated, target_type, center




def process_image_optimized(image_path, model_path, output_path=None):
    """
    Process a single image file with improved visualization for color detection and shape analysis

    Args:
        image_path: Path to input image
        model_path: Path to YOLO segmentation model
        output_path: Path to save the processed image (default: None, won't save)
    """
    # Check if CUDA is available and set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    (f"Using device: {device}")

    # Force CUDA if available
    if torch.cuda.is_available():
        torch.cuda.set_device(0)  # Use first GPU
        (f"CUDA device selected: {torch.cuda.get_device_name(0)}")
        (f"Available CUDA devices: {torch.cuda.device_count()}")
    else:
        ("WARNING: CUDA is not available, using CPU. Performance may be slow.")

    # Load model and send to appropriate device
    (f"Loading YOLO segmentation model from {model_path}...")
    model = YOLO(model_path)
    model.to(device)  # Move model to CUDA device if available

    # Load image
    (f"Loading image from {image_path}...")
    frame = cv2.imread(image_path)

    if frame is None:
        ("Error: Could not read the image file.")
        return

    (f"Processing image for detection...")

    # FPS hesaplama
    start_time = time.time()
    result_frame = process_single_frame_optimized(frame, model)
    process_time = time.time() - start_time
    fps = 1.0 / process_time if process_time > 0 else 0

    # Create a side-by-side view
    h, w = frame.shape[:2]
    combined = np.zeros((h, w * 2, 3), dtype=np.uint8)
    combined[:, :w] = frame
    combined[:, w:] = result_frame

    # Add labels

    # Sağ üst köşeye FPS değerini ekle

    # Save processed image if output path is provided
    if output_path:
        cv2.imwrite(output_path, result_frame)
        (f"Processed image saved to: {output_path}")

        # Also save the combined view
        combined_path = output_path.replace('.jpg', '_combined.jpg').replace('.png', '_combined.png')
        cv2.imwrite(combined_path, combined)
        (f"Combined view saved to: {combined_path}")

    # Create a resizable window with 1920x1080 dimensions
    cv2.namedWindow("Results (Original | Processed)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Results (Original | Processed)", 1920, 1080)

    # Display the result
    cv2.imshow("Results (Original | Processed)", combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    (f"Processing took {process_time:.3f} seconds ({fps:.1f} FPS)")

    return result_frame, combined


if __name__ == "__main__":
    # Set CUDA device options - Force CUDA usage if available
    if torch.cuda.is_available():
        torch.cuda.set_device(0)  # Use first GPU
        (f"CUDA device selected: {torch.cuda.get_device_name(0)}")
        (f"Available CUDA devices: {torch.cuda.device_count()}")
    else:
        ("WARNING: CUDA is not available, using CPU. Performance will be significantly reduced.")

    # File paths - you may need to update these
    # video_path = r'DJI_0348.MP4'
    video_path = 0
    image_path = r"uiha_sample/Screenshot_4.jpg"
    model_path = r'seg_best.pt'

    # Command line argument parsing
    parser = argparse.ArgumentParser(description='Process video/image to detect red triangles and blue hexagons')
    parser.add_argument('--mode', choices=['video', 'image'], default='video',
                        help='Processing mode: video or image')
    parser.add_argument('--start', type=float, default=255, help='Start time in seconds for video processing')
    parser.add_argument('--output', type=str, default=None, help='Output file path (only used for image mode)')
    args = parser.parse_args()

    # Print GPU information at startup
    if torch.cuda.is_available():
        ("===== CUDA INFORMATION =====")
        (f"CUDA available: Yes")
        (f"CUDA device count: {torch.cuda.device_count()}")
        (f"Current CUDA device: {torch.cuda.current_device()}")
        (f"Current CUDA device name: {torch.cuda.get_device_name(torch.cuda.current_device())}")
        (f"CUDA device properties:")
        for i in range(torch.cuda.device_count()):
            props = torch.cuda.get_device_properties(i)
            (f"  Device {i}: {props.name}")
            (f"    Total memory: {props.total_memory / 1024 ** 3:.2f} GB")
            (f"    CUDA Capability: {props.major}.{props.minor}")
        ("===========================")
    else:
        (
            "WARNING: CUDA is not available. Processing will be slow. Please check your GPU drivers and PyTorch installation.")

    # Run processing based on selected mode
    if args.mode == 'video':
        # Process video without saving output - only display
        process_video_optimized(video_path, model_path, start_second=args.start)
    else:
        # Process image with optional saving
        process_image_optimized(image_path, model_path, output_path=args.output)
