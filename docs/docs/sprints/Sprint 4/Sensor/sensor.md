---
title: Sensor de Temperatura
position: 1
---
# Sensor de Temperatura

&emsp;O presente documento tem como objetivo detalhar o sensor de temperatura utilizado no projeto Cannabot. O sensor de temperatura é um componente essencial para o monitoramento e controle do ambiente de cultivo, permitindo que o sistema possa realizar ajustes e tomar decisões com base nas informações coletadas.

&emsp;O sensor de temperatura utilizado é o modelo BME/BMP280, um sensor que mede a temperatura, pressão e umidade do ambiente. O sensor é conectado ao Raspberry Pi 4 por 4 jumpers, e realizamos a interface com ele usando a biblioteca de python `RPi.bme280 v0.2.4`.

## Nó de ROS

&emsp;Para a integração do sensor de temperatura com o ROS, foi criado um nó que publica a temperatura lida pelo sensor ao tópico ROS `/sensor_data`. O nó foi desenvolvido em Python e utiliza a biblioteca `RPi.bme280 v0.2.4` para realizar a leitura da temperatura (como mencionado anteriormente) além da biblioteca `smbus v0.3.4` para a interface com a pinagem do RPi.

&emsp;O nó é responsável por realizar a leitura da temperatura, umidade e pressão a cada 2 segundos e publicar o valor lido no tópico `/sensor_data`. O valor é publicado em um objeto do tipo `String`, porem nada mais e do que um JSON "stringified".

### Estrutura do JSON

```json
{
    "temperature_celsius": <float>,
    "temperature_fahrenheit": <float>,
    "pressure": <float>,
    "humidity": <float>
}
```

&emsp;Onde:

- `temperature_celsius`: Temperatura em graus Celsius.
- `temperature_fahrenheit`: Temperatura em graus Fahrenheit.
- `pressure`: Pressão atmosférica.
- `humidity`: Umidade relativa do ar.

## Conexão ao Backend

&emsp;Para a conexão do sensor de temperatura ao backend, o programa de websocket principal do robô, que expõe uma rota de websocket para controle e monitoramento foi ampliado, integrando mais um "subscriber" para assim publicar as informações extraídas a uma interface acessível de fora do robô.