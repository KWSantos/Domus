import asyncio
import websockets
import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = "yolov8n.pt"

try:
    model = YOLO(MODEL_PATH)
except Exception as e:
    print(f"Erro ao carregar o modelo: {e}")
    exit()

URI = "ws://192.168.0.111:81"
WINDOW_NAME = "Detecção de Objetos"

TOLERANCE = 30

async def control_robot(websocket, frame, results):
    frame_width = frame.shape[1]
    frame_center = frame_width // 2

    if results and results[0].boxes:
        box = results[0].boxes[0]
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        object_center_x = (x1 + x2) / 2

        offset = object_center_x - frame_center

        if abs(offset) < TOLERANCE:
            command = "MOVE_FORWARD"
        elif offset < 0:
            command = "TURN_LEFT"
        else:
            command = "TURN_RIGHT"

        await websocket.send(command)
        print(f"Comando enviado: {command}")
    else:
        await websocket.send("STOP")
        print("Nenhum objeto detectado, enviando STOP")

async def receive_frames():
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    async with websockets.connect(URI, max_size=2 ** 23) as websocket:
        try:
            while True:
                data = await websocket.recv()

                np_data = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
                frame = cv2.resize(frame, (320, 320))

                if frame is None:
                    continue

                results = model.predict(source=frame, conf=0.4, save=False, device='cuda' if model.device.type == 'cuda' else 'cpu')
                annotated_frame = results[0].plot()
                cv2.imshow(WINDOW_NAME, annotated_frame)

                await control_robot(websocket, frame, results)

                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except Exception as e:
            print(f"Erro no processamento do frame: {e}")

        finally:
            cv2.destroyAllWindows()
            print("Conexão encerrada e janelas fechadas.")

if __name__ == "__main__":
    try:
        asyncio.run(receive_frames())
    except KeyboardInterrupt:
        print("Interrupção pelo usuário.")
    finally:
        cv2.destroyAllWindows()
