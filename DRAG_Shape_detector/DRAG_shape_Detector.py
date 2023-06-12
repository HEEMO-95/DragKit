from ultralytics import YOLO


class Shape_Model:
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
    def __init__(self, model_path, conf=0.25):
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

                cropped_frame = frame[y1:y2, x1:x2]
                self.detections.append(
                    (self.class_Labels[class_id], cropped_frame, score, x1, x2, y1, y2)
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
                if class_id == self.desired_shape:
                    cropped_frame = frame[y1:y2, x1:x2]
                    self.detections.append(
                        (
                            self.class_labels[class_id],
                            cropped_frame,
                            score,
                            x1,
                            x2,
                            y1,
                            y2,
                        )
                    )

        return self.detections
