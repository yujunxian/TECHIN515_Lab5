from flask import Flask, request, jsonify
from tensorflow.keras.models import load_model
import numpy as np

app = Flask(__name__)

# Load model once at startup
model = load_model("wand_model.h5")
gesture_labels = ["V", "O", "Z", "S"]  # Update this to match your implementation

@app.route("/", methods=["GET"])
def home():
    return "Wand Gesture API is running!"

@app.route("/predict", methods=["POST"])
def predict():
    try:
        data = request.json.get("data")
        if not data:
            raise ValueError("Missing 'data' field")

        input_array = np.array(data).reshape(1, -1)  # Reshape for model input
        prediction = model.predict(input_array)

        top_index = int(np.argmax(prediction))
        label = gesture_labels[top_index]
        confidence = float(prediction[0][top_index]) * 100

        return jsonify({
            "gesture": label,
            "confidence": confidence
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 400

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)