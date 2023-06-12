from Object import Object
import cv2
import numpy as np
import math
from collections import Counter
import pytesseract
from ultralytics import YOLO

colors ={'green':(np.array([60,50,50]) , np.array([90,250,250])),
'red':(np.array([0,50,50]) , np.array([30,250,250])),
'blue':(np.array([120,50,50]) , np.array([150,250,250])),
'yellow':(np.array([30,50,50]) , np.array([60,250,250])),
'white':(np.array([0,0,255]) , np.array([255,255,255])),
'black':(np.array([0,0,0]) , np.array([255,255,0])),
'gray':(np.array([0,0,0]) , np.array([0,0,255])),
'purple':(np.array([130,0,0]) , np.array([140,255,255])),
'brown':(np.array([10,127,127]) , np.array([20,255,255])),
'orange':(np.array([8,127,127]) , np.array([22,255,255]))}

def crop_image(img, conts):
    images = []
    for boxes in conts.boxes.xyxy:
        x1, y1, x2, y2 = [int(box) for box in boxes]
        cropped_img = img.crop((x1, y1, x2, y2))
        images.append(cropped_img)
    return images


def rotate_image(image, angle):
    """
    Rotates an OpenCV 2 / NumPy image about it's centre by the given angle
    (in degrees). The returned image will be large enough to hold the entire
    new image, with a black background
    """

    # Get the image size
    # No that's not an error - NumPy stores image matricies backwards
    image_size = (image.shape[1], image.shape[0])
    image_center = tuple(np.array(image_size) / 2)

    # Convert the OpenCV 3x2 rotation matrix to 3x3
    rot_mat = np.vstack(
        [cv2.getRotationMatrix2D(image_center, angle, 1.0), [0, 0, 1]]
    )

    rot_mat_notranslate = np.matrix(rot_mat[0:2, 0:2])

    # Shorthand for below calcs
    image_w2 = image_size[0] * 0.5
    image_h2 = image_size[1] * 0.5

    # Obtain the rotated coordinates of the image corners
    rotated_coords = [
        (np.array([-image_w2, image_h2]) * rot_mat_notranslate).A[0],
        (np.array([image_w2, image_h2]) * rot_mat_notranslate).A[0],
        (np.array([-image_w2, -image_h2]) * rot_mat_notranslate).A[0],
        (np.array([image_w2, -image_h2]) * rot_mat_notranslate).A[0]
    ]

    # Find the size of the new image
    x_coords = [pt[0] for pt in rotated_coords]
    x_pos = [x for x in x_coords if x > 0]
    x_neg = [x for x in x_coords if x < 0]

    y_coords = [pt[1] for pt in rotated_coords]
    y_pos = [y for y in y_coords if y > 0]
    y_neg = [y for y in y_coords if y < 0]

    right_bound = max(x_pos)
    left_bound = min(x_neg)
    top_bound = max(y_pos)
    bot_bound = min(y_neg)

    new_w = int(abs(right_bound - left_bound))
    new_h = int(abs(top_bound - bot_bound))

    # We require a translation matrix to keep the image centred
    trans_mat = np.matrix([
        [1, 0, int(new_w * 0.5 - image_w2)],
        [0, 1, int(new_h * 0.5 - image_h2)],
        [0, 0, 1]
    ])

    # Compute the tranform for the combined rotation and translation
    affine_mat = (np.matrix(trans_mat) * np.matrix(rot_mat))[0:2, :]

    # Apply the transform
    result = cv2.warpAffine(
        image,
        affine_mat,
        (new_w, new_h),
        flags=cv2.INTER_LINEAR
    )
    image_rotated_cropped = crop_around_center(
        result,
        *largest_rotated_rect(
            image_size[0],
            image_size[1],
            math.radians(angle)
        )
    )

    return image_rotated_cropped


def largest_rotated_rect(w, h, angle):
    """
    Given a rectangle of size wxh that has been rotated by 'angle' (in
    radians), computes the width and height of the largest possible
    axis-aligned rectangle within the rotated rectangle.

    Original JS code by 'Andri' and Magnus Hoff from Stack Overflow

    Converted to Python by Aaron Snoswell
    """

    quadrant = int(math.floor(angle / (math.pi / 2))) & 3
    sign_alpha = angle if ((quadrant & 1) == 0) else math.pi - angle
    alpha = (sign_alpha % math.pi + math.pi) % math.pi

    bb_w = w * math.cos(alpha) + h * math.sin(alpha)
    bb_h = w * math.sin(alpha) + h * math.cos(alpha)

    gamma = math.atan2(bb_w, bb_w) if (w < h) else math.atan2(bb_w, bb_w)

    delta = math.pi - alpha - gamma

    length = h if (w < h) else w

    d = length * math.cos(alpha)
    a = d * math.sin(alpha) / math.sin(delta)

    y = a * math.cos(gamma)
    x = y * math.tan(gamma)

    return (
        bb_w - 2 * x,
        bb_h - 2 * y
    )


def crop_around_center(image, width, height):
    """
    Given a NumPy / OpenCV 2 image, crops it to the given width and height,
    around it's centre point
    """

    image_size = (image.shape[1], image.shape[0])
    image_center = (int(image_size[0] * 0.5), int(image_size[1] * 0.5))

    if (width > image_size[0]):
        width = image_size[0]

    if (height > image_size[1]):
        height = image_size[1]

    x1 = int(image_center[0] - width * 0.5)
    x2 = int(image_center[0] + width * 0.5)
    y1 = int(image_center[1] - height * 0.5)
    y2 = int(image_center[1] + height * 0.5)

    return image[y1:y2, x1:x2]


def id_ocr(img):
    ocr_config = r'--oem 1 --psm 10 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
    detected_ids = []
    for angle in range(0, 360, 10):
        rotated = rotate_image(img, angle)

        data = pytesseract.image_to_data(rotated, lang='eng', config=ocr_config, output_type='dict')
        conf = max(data['conf'])
        hci = data['conf'].index(conf)
        text = data['text'][hci]
        if conf > 80:
            detected_ids.append(text)
    frequency = Counter(detected_ids)
    most_frequent = frequency.most_common(1)

    most_conf = most_frequent[0][0]
    return most_conf


def detect_ids(shapes):
    data_list = []

    for shape in shapes:
        # RGB to BGR to use OpenCv
        img_np = np.array(shape)
        img_cv = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

        data = id_ocr(img_cv)
        data_list.append(data)

    return data_list


def id_detector(orig_img, boxes):
    shapes = crop_image(orig_img, boxes)
    return detect_ids(shapes)


def add_offset_to_contour(contour, offset):
    # Find the bounding rectangle of the contour with the maximum area
    x, y, w, h = cv2.boundingRect(contour)
    x -= offset
    y -= offset
    w += offset * 2
    h += offset * 2

    if w > h:
        offset = w - h
        y -= offset // 2
        h += offset
    else:
        offset = h - w
        x -= offset // 2
        w += offset

    return x, y, w, h

def eliminate_background(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)

    # Apply Otsu's thresholding
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_contour = max(contours, key=cv2.contourArea)
    mask = np.zeros_like(img)
    shape = cv2.drawContours(mask.copy(), [max_contour], -1, (255, 255, 255), -1)
    res = cv2.bitwise_and(img.copy(), shape.copy())

    return res


def get_roi(img):
    fimg = eliminate_background(img)

    lab = cv2.cvtColor(fimg, cv2.COLOR_BGR2LAB)
    lab = cv2.GaussianBlur(lab, (3, 3), 0)
    thresh = cv2.adaptiveThreshold(lab[:, :, 1], 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 27, 0)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    grandsons = []
    for i, con in enumerate(zip(contours, hierarchy[0])):
        is_root = con[1][1] == -1
        child_i = con[1][2]
        if is_root:
            while child_i > -1:
                grandson_i = hierarchy[0][child_i][2]
                while grandson_i > -1:
                    grandsons.append(contours[grandson_i])
                    grandson_i = hierarchy[0][grandson_i][0]
                child_i = hierarchy[0][child_i][0]
    if grandsons == [] :
        return img
    max_grandson = max(grandsons, key=cv2.contourArea)
    x, y, w, h = add_offset_to_contour(max_grandson, 30)

    return img[y:y + h, x:x + w]


def color_detection(frame):
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    frame = frame.reshape((-1,3))	
    # convert to np.float32
    frame = np.float32(frame)

    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 2
    ret,label,center = cv2.kmeans(frame,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    # Now convert back into uint8, and make original image
    center = np.uint8(center)
    res = center[label.flatten()]
    #cv.imshow('00',res2)
    #cv.waitKey(0)
    #Scv.destroyAllWindows()
    _,count = np.unique(label.flatten(),return_counts=True)
    center_idx = np.argmax(count)
    color_arr = center[center_idx]
    color = 'hhhhhh'
    for k,val in colors.items():
        h = color_arr[0]
        s = color_arr[1]
        v = color_arr[2]
        if (h < val[1][0]) and (h > val[0][0]) and (s < val[1][1]) and (s > val[0][1]) and (v < val[1][2]) and (v > val[0][2]):
            color = k
            break
    return color,color_arr
	
def alphabetic_detection(img):
    roi = get_roi(img)
    charc = id_ocr(roi)
    return charc

class Detector:
    """
    Shape_Model class for performing object detection using the YOLO model.

    Args:
        model_path (str): Path to the YOLO model file.
        conf (float): Confidence threshold for object detection.

    Attributes:
        model (YOLO): YOLO model instance for object detection.
        desired_shape (str or None): Desired shape label for region of interest.
        conf (float): Confidence threshold for object detection.

    Methods:
        set_conf: Set the confidence threshold for object detection.
        set_desired_shape: Set the desired shape to be in region of interest.
        get_conf: Get the confidence threshold for object detection.
        get_desired_shape: Get the desired shape label for region of interest.
        predict: Perform object detection on a frame.
        predict_desired_shape: Perform object detection on a frame with specified desired shape label.
    """

    # Constructor
    def __init__(self, model_path = '/DRAG_Shape_detector/runs/detect/train/weights/best.onnx', conf=0.25):
        self.model = YOLO(model_path)
        self.desired_shape = None
        self.conf = conf
        self.class_Labels = [
            "circle",
            "cross",
            "heptagon",
            "hexagon",
            "manikin",
            "octagon",
            "pentagon",
            "quartercircle",
            "rectangle",
            "semicircle",
            "square",
            "star",
            "trapezoid",
            "triangle",
        ]
        self.detections = []

    def set_conf(self, conf):
        """
        Set the confidence threshold for object detection.

        Args:
            conf (float): Confidence threshold for object detection.
        """
        self.conf = conf

    def set_desired_shape(self, desired_shape):
        """
        Set the desired shape label for region of interest.

        Args:
            desired_shape (str or None): Desired shape label for region of interest.
        """
        self.desired_shape = self.class_Labels.index(desired_shape)

    def get_conf(self):
        """
        Get the confidence threshold for object detection.

        Returns:
            float: Confidence threshold for object detection.
        """
        return self.conf

    def get_desired_shape(self):
        """
        Get the desired shape label for region of interest.

        Returns:
            str or None: Desired shape label for region of interest.
        """
        return self.desired_shape

    def predict(self, frame, stream=False, show=False):
        """
        Perform object detection on a frame.

        Args:
            frame (numpy.ndarray): Input frame for object detection.

        Returns:
            List[DetectionResult]: List of detection results.
        """
        results = self.model.predict(
            source=frame, stream=stream, show=show, conf=self.conf
        )
        for result in results:
            boxes = result.boxes  # Boxes object for bbox outputs
            for box in boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = box[:6]
                x1, y1, x2, y2, class_id = (
                    int(x1),
                    int(y1),
                    int(x2),
                    int(y2),
                    int(class_id),
                )
                x,y = ((x2+x1)/2),((y2+y1)/2)
                cropped_frame = frame[y1:y2, x1:x2]
                self.detections.append(
                    (self.class_Labels[class_id], cropped_frame, score,x,y)
                )

        return self.detections

    def predict_desired_shape(self, frame, shape):
        """
        Perform object detection on a frame with specified desired shape label.

        Args:
            frame (numpy.ndarray): Input frame for object detection.
            shape (str or None): Desired shape label for region of interest.

        Returns:
            List[DetectionResult]: List of detection results.
        """
        self.desired_shape = self.class_Labels.index(shape)

        results = self.model.predict(
            source=frame, stream=False, show=True, conf=self.conf
        )
        for result in results:
            boxes = result.boxes  # Boxes object for bbox outputs
            for box in boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = box[:6]
                x1, y1, x2, y2, class_id = (
                    int(x1),
                    int(y1),
                    int(x2),
                    int(y2),
                    int(class_id),
                )
                x,y = ((x2+x1)/2),((y2+y1)/2)
                if class_id == self.desired_shape:
                    cropped_frame = frame[y1:y2, x1:x2]
                    self.detections.append(
                        (
                            self.class_labels[class_id],
                            cropped_frame,
                            score,
                            x,
                            y
                        )
                    )

        return self.detections

def object_detection(frame):
    #pytesseract.pytesseract.tesseract_cmd = r'C:/Users/TFgam/AppData/Local/Tesseract-OCR/tesseract.exe'
    cls_names = model.names
    print(cls_names)
    img,probs,shape,x,y = yolo(frame)
    color = color_detection(img)
    #charc = alphabetic_detection(img)
    obj = Object(color,int(shape),'charc',(x,y))
    return obj,probs
