
from flask import Flask, render_template_string, Response
import cv2
from ultralytics import YOLO

app = Flask(__name__)

# Chargement du modèle YOLOv8 (ici yolov8n, léger)
model = YOLO("/home/popeye/best.pt")

# Capture webcam (change l’index si nécessaire)
camera = cv2.VideoCapture("/dev/video1")

# Générateur de flux MJPEG avec inférence YOLOv8
def generate():
    while True:
        success, frame = camera.read()
        if not success:
            break

        # Inférence avec YOLOv8
        results = model(frame, verbose=False)[0]

        # Dessiner les bounding boxes
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            conf = box.conf[0]
            label = f"{model.names[cls]} {conf:.2f}"

            # Dessin
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Encode frame en JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # Flux MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template_string("""
    <html>
      <head>
        <title>YOLOv8 Stream</title>
      </head>
      <body>
        <h1>Stream avec YOLOv8</h1>
        <img src="{{ url_for('video_feed') }}" width="640" height="480">
      </body>
    </html>
    """)

@app.route('/video_feed')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
