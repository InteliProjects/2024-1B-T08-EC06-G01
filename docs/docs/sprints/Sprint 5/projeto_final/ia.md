---
title: "Modelo de Visão Computacional"
sidebar_position: 6
description: Nessa secção será detalhado como está a implementação do modelo de machine learning utilizando visão computacional.
---

## Introdução

&emsp;O modelo de visão computacional é um elemento importante no projeto. A implementação foi feita pensando em diminuir a carga cognitiva para o operador do robô. Assim, a ia é capaz de detectar determinado objeto estranho que possa estar dentro do reboiler durante a operação. Nesta secção estarão documentados o processo de treinamento do modelo, dataset utilizado, implementação e próximos passos.

## Modelo
&emsp;O modelo utilizado foi um modelo de visão computacional que fosse rápido em conseguir detectar objetos estranhos dentro do reboiler. Assim, utilizamos o modelo [YOLO V8](https://github.com/ultralytics/ultralytics). Esse é um modelo muito utilizado por diversas pessoas e empresas ao redor do globo. Possui uma ampla documentação e pode ser aplicado em diversos tipos de problemas que requerem o uso de visão computacional.

## Dataset e Treinamento

#### Dataset
&emsp;O problema de detectar objetos que possam estar dentro do reboiler é complexo, pensando que o robô está sendo desenvolvido com o objetivo de funcionar dentro de um ambiente escuro e inóspito para as maiorias das formas de vida.O modelo de ia tem que ser capaz de detectar possíveis corpos estranhos. 

&emsp;Com esse problema em mente a solução foi encontrada utilizando um dataset que serve para identificar objetos estranhos em tubulações de esgoto. Para mais informações sobre o mesmo é possível saber mais no seguinte [link](https://universe.roboflow.com/purdue-university-niruh/precision-ag-subterranean) 


#### Treinamento
&emsp;Para treinar o modelo foi utilizado uma instância do Google Colab com uma gpu T4. O motivo da utilização do colab foi seu ambiente fácil de desenvolvimento e a capacidade de suprir as demandas computacionais em um intervalo de tempo satisfatório. 

&emsp;O processo de treinamento do modelo ao utilizar a Yolo-V8 é muito simples, só basta instalar a biblioteca da ultralytics, definir um dataset e começar a realizar o treinamento do modelo. 

```python
model = YOLO("yolov8n.yaml")
results = model.train(data='../dataset/dirt_detection/data.yaml', epochs=100)
```

&emsp;Com duas linhas de código o modelo já está sendo treinado. Após essa etapa só basta exportar o modelo e começar a utilizar o mesmo. Para mais detalhes de como o modelo foi implementado é recomendado acessar o notebook de treinamento. O mesmo está localizado no repositório do projeto no seguinte caminho: *src/ia/notebooks/model_creation.ipynb*.

Na figura abaixo é possível ver como é o resultado da detecção do modelo

<p align="center"><b> Detecção com yolo</b></p>
<div align="center">
  ![](../../../../static/img/sprint4/result.jpg)
  <p><b>Fonte:</b> Elaborado por Cannabot</p>
</div>


## Implementação do modelo 

&emsp;Após o treinamento do modelo é preciso utilizá-lo. Assim, utilizando a biblioteca da ultralytics, é possível enviar inputs para o modelo e o mesmo retorna se detectou um objeto estranho no reboiler. Aproveitando a conexão websocket os inputs da camera quando necessário, são enviados para o modelo de ia e o resultado é retornado no frontend. 
O modo que tal implementação foi feita pode ser visto com mais detalhes no arquivo:*src/backend/src/client/camera.py*.

O código que executa a detecção é o seguinte: 

```python
    async def process_message(self, message):
        data = json.loads(message)
        if "bytes" in data:
            img_data = base64.b64decode(data["bytes"])
            np_arr = np.frombuffer(img_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            results = self.yolo_model.predict(img)  # Rodar o modelo YOLO


            results = await asyncio.to_thread(self.yolo_model.predict, img)
            results = results[0]

            # Printar o resultado da IA
            print("Probs:", results.probs)
```

## Conclusão e próximos passos
&emsp;O modelo de IA foi desenvolvido com o objetivo de diminuir a carga cognitiva do usuário. Assim, ao utilizar a IA, o intuito é facilitar a vida do usuário final. O modelo está operando com a biblioteca da Ultralytics para realizar a detecção de objetos atípicos. No momento, conforme o projeto foi concebido, não há próximos passos para a IA, uma vez que a implementação atual já satisfaz plenamente as demandas.