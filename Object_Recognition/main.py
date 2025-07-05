import asyncio
import websockets
import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = "yolov8m.pt"
# ESP32_CAM_URI = "ws://10.42.0.2:81"
ESP32_CAM_URI = "ws://192.168.0.106:81"
# ESP32_WROVER_URI = "ws://10.42.0.3:81"
ESP32_WROVER_URI = "ws://192.168.0.107:81"
WINDOW_NAME = "Detecção de Objetos"

TOLERANCE = 25
CONFIDENCE_THRESHOLD = 0.4
TURN_ANGLE_INCREMENT = 5.0

model = YOLO(MODEL_PATH)
latest_frame = None
frame_lock = asyncio.Lock()
last_command_sent = "STOP"
is_robot_busy = False


current_operation_mode = "FINDER"
current_target_class = "bottle"

async def handle_finder_bring_mode(control_ws, target_box, frame_center):
    """Lógica para os modos FINDER e BRING, que é Alinhar e Avançar."""
    global is_robot_busy, last_command_sent

    x1, _, x2, _ = target_box.xyxy[0].cpu().numpy()
    object_center_x = (x1 + x2) / 2
    offset = object_center_x - frame_center

    command_to_send = "STOP"
    if abs(offset) > TOLERANCE:
        if offset < 0:
            command_to_send = f"TURN:-{TURN_ANGLE_INCREMENT}"
        else:
            command_to_send = f"TURN:{TURN_ANGLE_INCREMENT}"
    else:
        command_to_send = "MOVE_FORWARD"

    if command_to_send != last_command_sent:
        await control_ws.send(command_to_send)
        print(f"Modo: {current_operation_mode} | Alvo: {current_target_class} | Comando: {command_to_send}")
        last_command_sent = command_to_send
        if "TURN" in command_to_send:
            is_robot_busy = True


async def handle_follow_mode(control_ws, target_box, frame_center):
    """Lógica para o modo FOLLOW, que é apenas Alinhar (sem avançar)."""
    global is_robot_busy, last_command_sent

    x1, _, x2, _ = target_box.xyxy[0].cpu().numpy()
    object_center_x = (x1 + x2) / 2
    offset = object_center_x - frame_center

    command_to_send = "STOP"
    if abs(offset) > TOLERANCE:
        if offset < 0:
            command_to_send = f"TURN:-{TURN_ANGLE_INCREMENT}"
        else:
            command_to_send = f"TURN:{TURN_ANGLE_INCREMENT}"

    if command_to_send != last_command_sent:
        await control_ws.send(command_to_send)
        print(f"Modo: {current_operation_mode} | Alvo: {current_target_class} | Comando: {command_to_send}")
        last_command_sent = command_to_send
        if "TURN" in command_to_send:
            is_robot_busy = True

async def control_robot(control_ws, frame, results):
    """
    Função principal que atua como um roteador, chamando a lógica
    correspondente ao modo de operação atual.
    """
    global last_command_sent

    if is_robot_busy:
        return

    frame_center = frame.shape[1] // 2

    valid_boxes = [box for box in results[0].boxes if
                   model.names[int(box.cls[0].item())] == current_target_class and box.conf[
                       0].item() >= CONFIDENCE_THRESHOLD]

    if valid_boxes:
        target_box = max(valid_boxes, key=lambda box: box.conf[0].item())

        if current_operation_mode == "FINDER" or current_operation_mode == "BRING":
            await handle_finder_bring_mode(control_ws, target_box, frame_center)
        elif current_operation_mode == "FOLLOW":
            await handle_follow_mode(control_ws, target_box, frame_center)

    else:
        if last_command_sent != "STOP":
            await control_ws.send("STOP")
            print(f"Modo: {current_operation_mode} | Procurando por {current_target_class}... | Comando: STOP")
            last_command_sent = "STOP"

async def frame_receiver(cam_ws):
    global latest_frame
    while True:
        try:
            data = await cam_ws.recv()
        except websockets.ConnectionClosed:
            break
        async with frame_lock:
            latest_frame = data


async def robot_response_handler(control_ws):
    """Escuta por mensagens do robô para atualizar os estados e modos."""
    global is_robot_busy, last_command_sent, current_operation_mode, current_target_class
    while True:
        try:
            message = await control_ws.recv()
            print(f"<-- Mensagem recebida do robô: {message}")

            if message == "TURN_COMPLETE":
                is_robot_busy = False
                last_command_sent = None

            elif message == "GRAB_SEQUENCE_COMPLETE":
                if current_operation_mode == "FINDER":
                    current_operation_mode = "BRING"
                    current_target_class = "person"
                    is_robot_busy = False
                    last_command_sent = None

        except websockets.ConnectionClosed:
            break


async def inference_and_control(control_ws):
    """Loop principal de inferência, agora com controle de modo por teclado."""
    global latest_frame, current_operation_mode, current_target_class

    while True:
        await asyncio.sleep(0.05)
        async with frame_lock:
            if latest_frame is None: continue
            data = latest_frame;
            latest_frame = None
        if data:
            np_data = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
            if frame is None: continue
            frame = cv2.flip(frame, 0)
            frame_resized = cv2.resize(frame, (320, 320))
            results = model.predict(source=frame_resized, conf=CONFIDENCE_THRESHOLD, verbose=False)
            await control_robot(control_ws, frame_resized, results)

            annotated_frame = results[0].plot()
            cv2.imshow(WINDOW_NAME, annotated_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('f'):
                current_operation_mode = "FINDER"
                current_target_class = "bottle"
            elif key == ord('p'):
                current_operation_mode = "FOLLOW"
                current_target_class = "person"


async def main():
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    try:
        async with websockets.connect(ESP32_CAM_URI, max_size=2 ** 22) as cam_ws, \
                websockets.connect(ESP32_WROVER_URI) as control_ws:
            print("Conectado à Câmera e ao Robô.")
            await control_ws.send("STOP")

            global current_operation_mode, current_target_class
            current_operation_mode = "FINDER"
            current_target_class = "bottle"

            tasks = [
                frame_receiver(cam_ws),
                inference_and_control(control_ws),
                robot_response_handler(control_ws)
            ]
            await asyncio.gather(*tasks)

    except Exception as e:
        print(f"Erro principal: {e}")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nEncerrado pelo usuário.")
    finally:
        cv2.destroyAllWindows()