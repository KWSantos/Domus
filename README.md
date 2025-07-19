# Domus - Robô de Assistência Doméstica Autônomo

![Status do Projeto](https://img.shields.io/badge/status-concluído-brightgreen)
![Linguagem Principal](https://img.shields.io/github/languages/top/KWSantos/Domus?color=blue)
![Licença](https://img.shields.io/badge/license-MIT-blue)

Repositório do Trabalho de Conclusão de Curso (TCC) para o desenvolvimento do **Domus**, um protótipo de robô de baixo custo voltado para a assistência doméstica.

## 🎥 Demonstração

O vídeo abaixo demonstra as principais funcionalidades do robô em ação: busca e coleta de objetos e o acompanhamento de um alvo humano.

[![Demonstração do Robô Domus](https://i.ytimg.com/vi/rp_-HHrAA_I/mqdefault.jpg)](https://youtu.be/rp_-HHrAA_I)
> **Clique na imagem para assistir ao vídeo de demonstração no YouTube**

---

## 📖 Índice

* [Sobre o Projeto](#-sobre-o-projeto)
* [Funcionalidades](#-funcionalidades)
* [Arquitetura do Sistema](#-arquitetura-do-sistema)
* [Tecnologias Utilizadas](#-tecnologias-utilizadas)
* [Resultados](#-resultados)
* [Estrutura do Repositório](#-estrutura-do-repositório)
* [Como Executar](#-como-executar)
* [Autor](#-autor)

---

## 🎯 Sobre o Projeto

O projeto **Domus** nasceu da crescente demanda por tecnologias que promovam o bem-estar e a autonomia de pessoas com mobilidade reduzida, como idosos e pessoas com deficiência. O objetivo foi desenvolver um protótipo de robô assistente que fosse funcional, seguro e, principalmente, de **baixo custo** (custo total aproximado de R$ 350), utilizando componentes acessíveis e software de código aberto.

O robô é capaz de navegar de forma autônoma em um ambiente doméstico, identificar objetos e pessoas com visão computacional e executar tarefas de manipulação, como buscar e entregar itens.

---

## ✨ Funcionalidades

* **Busca e Coleta de Objetos**: O robô pode localizar um objeto específico (ex: uma garrafa), navegar até ele, pegá-lo com sua garra e retornar à origem.
* **Seguir Pessoa**: O robô pode identificar uma pessoa e segui-la, mantendo uma distância segura e acompanhando seus movimentos.
* **Controle Remoto e Tomada de Decisão Autônoma**: O sistema opera com uma arquitetura distribuída, onde a tomada de decisão baseada em IA ocorre em um computador host, que comanda as ações físicas do robô.

---

## 🛠️ Arquitetura do Sistema

A operação do Domus é governada por uma arquitetura de software distribuída em três módulos que se comunicam via **WebSockets** em uma rede Wi-Fi local.

1.  **Módulo de Percepção (ESP32-CAM)**
    * **Responsabilidade**: Atuar como os "olhos" do robô.
    * **Funcionamento**: Captura um fluxo de vídeo contínuo e o transmite para o computador host.
    * **Hardware**: Placa ESP32-CAM com câmera.

2.  **Módulo de Decisão (Computador Host)**
    * **Responsabilidade**: O "cérebro" do sistema.
    * **Funcionamento**: Recebe o vídeo, executa um modelo de rede neural **YOLOv8** para detecção de objetos em tempo real, interpreta a cena e envia comandos de alto nível (ex: "vire à direita", "avance") para o robô.
    * **Software**: Script em Python com OpenCV e Ultralytics YOLO.

3.  **Módulo de Atuação (ESP32-WROVER)**
    * **Responsabilidade**: Os "músculos" do robô, responsável por traduzir comandos em ações concretas.
    * **Funcionamento**: Recebe os comandos do host e os traduz em ações físicas, controlando os motores DC para locomoção e o servomotor da garra. Utiliza uma **Máquina de Estados Finitos (FSM)** para gerenciar sequências de ações e um sensor ultrassônico para evitar colisões.
    * **Hardware**: Placa ESP32-Wrover, Ponte H L298N, Motores DC, Servomotor, Sensor Ultrassônico HC-SR04.
---

## 🚀 Tecnologias Utilizadas

#### Hardware
* **Microcontroladores**: ESP32-CAM e ESP32-WROVER
* **Chassi**: Estrutura em MDF baseada no projeto open-source [PioneerDS](https://mauriciodgsantos.wixsite.com/easyds/pioneerds)
* **Atuadores**: 4x Motores DC com caixa de redução, Driver Ponte H L298N, Micro Servomotor SG90
* **Sensores**: Câmera OV2640 (integrada ao ESP32-CAM), Sensor Ultrassônico HC-SR04, IMU MPU6050

#### Software (Computador Host)
* **Linguagem**: Python
* **Bibliotecas Principais**:
    * `OpenCV`: Para processamento de imagem em tempo real.
    * `Ultralytics YOLOv8`: Para detecção de objetos.
    * `websockets`, `asyncio`: Para comunicação assíncrona com os microcontroladores.

#### Software (Embarcado)
* **Linguagem**: C/C++ (Framework Arduino)
* **IDE**: Arduino IDE
* **Bibliotecas Principais**: `ArduinoWebsockets`

---

## 📊 Resultados

Os testes de desempenho validaram a eficácia do protótipo. O destaque foi a comparação entre o modelo YOLOv8 pré-treinado e uma versão customizada, treinada com um dataset de objetos domésticos.

* **Taxa de Sucesso (Missão 'Buscar e Trazer')**:
    * Modelo Pré-treinado: **80%**
    * Modelo Customizado: **100%**
* **Eficiência**: O modelo customizado não só eliminou as falhas da missão, como também reduziu o tempo médio de conclusão em **14%**.

Esses dados confirmam que o *fine-tuning* do modelo de visão computacional é crucial para a robustez e eficiência em aplicações de robótica específicas.

---

## 📂 Estrutura do Repositório

    .
    ├── Documentação/             # Contém o relatório final do TCC em .pdf
    ├── Object_Recognition/       # Código Python do computador host (módulo de decisão)
    ├── Robot_Control/            # Código C++/Arduino para o ESP32-WROVER (módulo de atuação)
    ├── Video_Transmission/       # Código C++/Arduino para o ESP32-CAM (módulo de percepção)
    ├── .gitignore
    └── README.md

* **`Documentação`**: Contém o relatório completo do TCC (`Relatório_TCC_Kaue.pdf`), que detalha toda a fundamentação teórica, metodologia, desenvolvimento e resultados do projeto.
* **`Object_Recognition`**: Script Python que deve ser executado no computador host. Ele recebe o vídeo, processa com YOLO e envia comandos para o robô.
* **`Robot_Control`**: Firmware para a placa ESP32-Wrover, responsável por controlar os motores e a garra.
* **`Video_Transmission`**: Firmware para a placa ESP32-CAM, responsável por capturar e transmitir o vídeo via WebSocket.

---

## 🚀 Como Executar

#### Pré-requisitos
* Hardware montado conforme o esquemático do projeto.
* [Arduino IDE](https://www.arduino.cc/en/software) instalado com o suporte para placas ESP32.
* [Python 3.8+](https://www.python.org/downloads/) instalado no computador host.

#### 1. Configuração do Ambiente Host

    # Clone o repositório
    git clone https://github.com/KWSantos/Domus.git
    cd Domus/Object_Recognition

    # Crie um ambiente virtual e ative-o (recomendado)
    python -m venv venv
    
    # No Windows:
    venv\Scripts\activate
    
    # No Linux/macOS:
    source venv/bin/activate

    # Instale as dependências
    pip install opencv-python ultralytics websockets

#### 2. Gravação dos Firmwares
1.  Abra o Arduino IDE.
2.  Carregue o código de `Video_Transmission/Video_Transmission.ino` em sua placa **ESP32-CAM**.
    * **Atenção**: Configure suas credenciais de Wi-Fi (SSID e senha) e o endereço IP estático no código antes de gravar.
3.  Carregue o código de `Robot_Control/Robot_Control.ino` em sua placa **ESP32-WROVER**.
    * **Atenção**: Configure suas credenciais de Wi-Fi e o endereço IP estático no código antes de gravar.

#### 3. Execução
1.  Ligue o robô. As duas placas ESP32 se conectarão à sua rede Wi-Fi.
2.  No seu computador host (com o ambiente virtual ativado), execute o script de controle.
3.  A janela do OpenCV deve abrir, exibindo a visão do robô com as detecções do YOLO. O robô começará a operar de acordo com a lógica programada.

---

## ✍️ Autor

* **Kauê Santos da Cruz** - [GitHub: KaweSC02](https://github.com/KWSantos)

Projeto desenvolvido para o curso de Ciência da Computação da Universidade Estadual do Centro-Oeste (UNICENTRO).
