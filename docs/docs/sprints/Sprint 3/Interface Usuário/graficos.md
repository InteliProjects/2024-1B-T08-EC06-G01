---
title: "Sugestão de gráficos"
sidebar_position: 1
---

# Gráficos 

&emsp;O projeto desenvolvido pelo grupo cannabot conta com um robô turtlebot3 que tem a possibilidade de ser acoplado a um micro-controlador raspeberry pi pico. Essa conexão com o Raspeberry permite a conexão de um sensor de temperatura e gerar um fluxo de dados que podem ser visualizados. 

&emsp;O grupo pensou em alguns possíveis gráficos que podem ser desenvolvidos e que trariam grande vantagem na parte da inteligência do negócio. Permitindo enxergar padrões e tendências no funcionamento do reboiler. 

### Mapa de calor (Heatmap) 

&emsp;Este é o principal gráfico a ser utilizado em toda solução. Ele permite visualizar por cima do reboiler como anda a temperatura em cada ponto específico. Esse gráfico tem como objetivo ser gerado em tempo real, conforme o robô vai coletando os dados o gráfico vai sendo atualizado e o operador do robô entende como anda a situação do reboiler. 

&emsp;Na figura abaixo é possível ver um exemplo de heatmap e como foi pensado a sua implementação.

<div align="center"> 

![](../../../../..\docs\static\img\sprint3\heatmap.png)

Fonte: Elaborado pelo grupo Cannabot
</div>


### Histórico de temperaturas diária 

&emsp;Outro gráfico que pode ser gerado é um análise diária dos setores do reboilers. Assim, é possível dividir o reboiler, que possui um formato de círculo, em oito quadrantes e cada um desses quadrantes corresponde a uma barra no gráfico que apresenta a média da temperatura. 

&emsp;Assim, o operador do sistema consegue ter uma noção se a temperatura dentro do reboiler é homôgenea ou há variação em algum setor específico. Com base nessas informações é possível tomar uma decisão mais assertiva se é hora ou não de limpar o reboiler. 

&emsp;Na figura abaixo é possível ver o esquema de como é feita a divisão do reboiler em quadrantes e o gráfico com a média de temperatura para cada região.

<div align="center"> 

![Divisão do Reboiler](../../../../..\docs\static\img\sprint3\circle.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

<div align="center"> 

![Média da temperatura por setor](../../../../..\docs\static\img\sprint3\grafico1.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

### Gráfico de Barras

&emsp;Ademais, considerando todos os reboilers da planta como método de anáise, através de um gráfico de barras, é possível monitorar as variações de  temperaturas mínimas, médias e máximas para diferentes reboilers.
Na figura abaixo é possível ver um exemplo simples de um gráfico de barras

<div align='center'>

![Média da temperatura por reboiler](../../../../..\docs\static\img\sprint3\bars.png)
Fonte: Elaborado pelo grupo Cannabot
</div>


### Gráfico de Dispersão (Scatter Plot)

&emsp;Um gráfico de dispersão pode ser utilizado para correlacionar a temperatura com outros fatores, como a umidade. Este gráfico ajuda a identificar possíveis relações entre a temperatura dos reboilers e a umidade ambiente, permitindo uma análise mais profunda sobre como as condições ambientais podem influenciar o desempenho dos reboilers.

<div align='center'>

![Scatter Plot (Temperartura x Humidade)](../../../../..\docs\static\img\sprint3\scatterplot.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

### Conclusão

&emsp;Os gráficos sugeridos pelo projeto Cannabot fornecem insights valiosos sobre o desempenho e as condições operacionais dos reboilers. O mapa de calor (Heatmap) permite um monitoramento em tempo real da distribuição de temperatura, enquanto o gráfico de histórico diário facilita a análise das variações de temperatura ao longo do tempo. O gráfico de barras oferece uma visão comparativa entre diferentes reboilers, e o scatter plot destaca as correlações entre temperatura e umidade.

&emsp;Essas ferramentas visuais ajudam os operadores a identificar padrões, detectar anomalias e tomar decisões informadas, otimizando a manutenção e a operação dos reboilers. Com a implementação desses gráficos, a plataforma de BI se torna uma ferramenta essencial para melhorar a eficiência e a produtividade na indústria de produção de açúcar e etanol.