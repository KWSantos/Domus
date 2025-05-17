import asyncio
import websockets
import cv2
import numpy as np
import torch

# Configura o modelo YOLOv5
try:
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    model.to('cuda' if torch.cuda.is_available() else 'cpu').eval()
    print("Modelo YOLOv5 carregado com sucesso!")
except Exception as e:
    print(f"Erro ao carregar o modelo: {e}")
    exit()

URI = "ws://192.168.3.73:81"
WINDOW_NAME = "Detecção de Objetos"


async def receive_frames():
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    async with websockets.connect(URI, max_size=2 ** 23) as websocket:
        try:
            while True:
                try:
                    data = await websocket.recv()

                    # Converte os bytes para imagem
                    np_data = np.frombuffer(data, dtype=np.uint8)
                    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

                    if frame is None:
                        print("⚠️ Frame inválido recebido.")
                        continue

                    # Realiza inferência
                    with torch.no_grad():  # Desativa gradientes para inferência
                        results = model(frame)

                    # Renderiza as detecções
                    results.render()
                    if len(results.ims) > 0:
                        annotated = results.ims[0]
                        cv2.imshow(WINDOW_NAME, annotated)

                    # Verifica se a janela ainda existe
                    if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                        break

                    # Delay para exibição e verificação de tecla 'q'
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                except Exception as e:
                    print(f"Erro no processamento do frame: {e}")
                    break

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