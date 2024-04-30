| Suporte para os Modelos  | ESP32 | ESP32-S3 |
| ------------------------ | ----- | -------- |
# PROJETO BASE PARA INTEGRAÇÃO ENTRE BLUETOOTH, PWM E UART EM RTOS

## Disciplina: Sistemas Embarcos em Tempo Real 

## Criadores do Projeto
### Vinicius Moirera Nascimento Ra° 734551

O "Projeto Base para Integração entre Bluetooth, PWM e UART em RTOS" é um plano de desenvolvimento que visa criar uma estrutura sólida para a interconexão de tecnologias-chave, como Bluetooth, PWM (Modulação por Largura de Pulso) e UART (Universal Asynchronous Receiver/Transmitter) em um ambiente de sistema operacional em tempo real (RTOS). Este projeto oferece uma base sólida para a construção de sistemas complexos que exigem comunicação sem fio, controle de dispositivos e sincronização de tarefas em tempo real, promovendo a eficiência e a confiabilidade em aplicações variadas.

## Como utilizar os exemplos

### Hardware


O "Software de Desenvolvimento Base" possui o principio de construir sistemas altamente personalizados e eficazes. Este conjunto de ferramentas oferece uma base sólida para o desenvolvimento de software mais especificos , permitindo a integração de recursos avançados e funcionalidades personalizadas. Abaixo, fornecemos uma breve descrição de como a integração com o hardware é alcançada:

Conexões para o PWM :

```
      ESP Board              Servo Motor      5V
+-------------------+     +---------------+    ^
|  SERVO_PULSE_GPIO +-----+PWM        VCC +----+
|                   |     |               |
|               GND +-----+GND            |
+-------------------+     +---------------+
```

Conexões para o Bluetooth :

```
      ESP Board              Dispositivo     
+-------------------+     +-------------------+    
|                   |     |                   |
| Bluetooth classic +-----+Bluetooth classic  |
|                   |     |                   |
+-------------------+     +-------------------+
```

Conexões para o UART :

```
      ESP Board                  ESP Board     
+-------------------+     +-------------------+    
|                   |     |                   |
|                TX +-----+RX                 |
|                RX +-----+TX                 |
|                   |     |                   |
+-------------------+     +-------------------+
```
