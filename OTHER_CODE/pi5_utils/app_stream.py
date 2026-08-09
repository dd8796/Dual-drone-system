from flask import Flask, Response, render_template_string
import cv2

app = Flask(__name__)

# Initialisation de la webcam (0 = première webcam)
camera = cv2.VideoCapture(0)

# Générateur de frames MJPEG
def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Encode la frame en JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # Envoie la frame en multipart
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Page HTML basique avec le stream intégré
@app.route('/')
def index():
    return render_template_string("""
        <html>
            <head>
                <title>Webcam Stream</title>
            </head>
            <body>
                <h1>Flux webcam en direct</h1>
                <img src="{{ url_for('video') }}" width="640" height="480">
            </body>
        </html>
    """)

# Route du flux vidéo
@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
