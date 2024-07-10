---
title: "Teste de requisito funcional"
position: 1
---

&emsp;Esta seção visa documentar os testes realizados em conformidade com o RF3 (Requisito Funcional 3), relacionado à interface gráfica.

&emsp;O requisito em questão foi definido da seguinte maneira:

**RF3 - Interface Gráfica**

- **Teleoperação do Robô:** A interface gráfica deve possibilitar a teleoperação completa do robô, permitindo sua movimentação precisa pelo reboiler por meio de comandos intuitivos. Ademais, deve incluir um botão de emergência de fácil acesso para interromper imediatamente a operação do robô em caso de necessidade.

- **Visualização em Tempo Real:** A interface deve oferecer uma visualização em tempo real das imagens capturadas pela câmera do robô, permitindo que os operadores acompanhem a operação de forma contínua. Essa visualização deve ser clara e detalhada, garantindo que os operadores possam movimentar o robô com precisão por meio da interface gráfica.

&emsp;Com o propósito de testar a funcionalidade mencionada anteriormente, foram elaborados os seguintes casos de teste, que têm o objetivo de destacar os pontos que devem ser observados para localizar possíveis melhorias, a fim de cumprir o requisito com sucesso. Esses casos de teste servem como base para a elaboração do roteiro de testes, que definirá os desafios que os usuários devem enfrentar para garantir que passem por todos os casos.

## Casos de teste

| Número do caso de teste | 01 |
|-------------------------|----|
| **Caso de teste**       | Testar o funcionamento dos botões de movimentação do robô |
| **Localização**         | Home > Central de Controle > Modo Teleoperado |
| **Pré-condições**       | 1. Ter realizado o cadastro no site <br/> 2. Ter efetuado o login no site |
| **Procedimento**        | 1. Entrar na Tela Home > Central de Controle > Modo Teleoperado <br/> 2. Pressionar as teclas de movimentação no teclado (setas de esquerda, direita, cima, baixo ou A,W,S,D) <br/> 3. Verificar se o robô se movimenta corretamente em todos os casos, e não colide com nenhum obstáculo ao redor |
| **Resultados Esperados**| Espera-se que o usuário utilize intuitivamente tanto as setas da interface quanto as setas do teclado para movimentar o robô. A movimentação do robô deve ser perceptível pelo usuário, e os comandos devem ser executados sem atraso. Quando o robô se aproximar de um obstáculo, ele não deve mais se mover na direção do obstáculo, evitando a colisão mecanicamente e acionando um pop-up de alerta na interface |

| Número do caso de teste | 02 |
|-------------------------|----|
| **Caso de teste**       | Testar o funcionamento do botão de emergência |
| **Localização**         | Home > Central de Controle > Modo Teleoperado |
| **Pré-condições**       | 1. Ter realizado o cadastro no site <br/> 2. Ter efetuado o login no site |
| **Procedimento**        | 1. Entrar na Tela Home > Central de Controle > Modo Teleoperado <br/> 2. Clicar no botão de “modo de emergência” <br/> 3. Clicar ou pressionar os botões de movimentação <br/> 4. Verificar se o robô continua se movimentando ao realizar o passo 3. |
| **Resultados Esperados**| Ao apertar o botão de emergência, espera-se que o usuário não consiga mais movimentar o robô, mesmo utilizando as setas de movimentação. A movimentação do robô só deve ser retomada após um reinício manual. |

| Número do caso de teste | 03 |
|-------------------------|----|
| **Caso de teste**       | Testar o funcionamento da visualização da câmera do robô |
| **Localização**         | Home > Central de Controle > Modo Teleoperado |
| **Pré-condições**       | 1. Ter realizado o cadastro no site <br/> 2. Ter efetuado o login no site |
| **Procedimento**        | 1. Entrar na Tela Home > Central de Controle > Modo Teleoperado <br/> 2. Checar se é possível visualizar o ambiente à frente do robô. <br/> 3. Movimentar o robô por meio das setas. <br/> 4. Checar se a atualização da câmera ocorre de forma estável e sem atraso ou travamento. |
| **Resultados Esperados**| Espera-se que o usuário consiga visualizar claramente a parte frontal do ambiente em que o robô está operando, além de acompanhar sua movimentação sem atrasos significativos. |


&emsp;Para garantir a verificação dos casos de teste delineados anteriormente, elaborou-se o seguinte roteiro de teste, aplicado a 4 usuários distintos, e os resultados foram registrados em cada caso.

## Roteiro de Teste

Olá, usuário!

&emsp;Este documento apresenta uma série de desafios elaborados para testar o funcionamento de uma das funcionalidades fundamentais de nossa solução, que envolve a operação de um robô por meio de uma interface gráfica e a detecção de dados como temperatura e localização.

&emsp;Abaixo, listamos cada desafio relacionado à funcionalidade básica que desejamos testar. Espera-se que os desafios sejam realizados na ordem apresentada. Caso encontre dificuldades em completar algum deles, sinta-se à vontade para prosseguir para o próximo.

&emsp;Durante o teste, duas pessoas estarão acompanhando você: uma com quem você interagirá e que explicará os desafios, e outra que registrará os resultados apresentados.

**Desafios**

1. Utilizando a interface, mova o robô em direção à marcação feita no chão.

2. Por meio da interface, movimente o robô em direção a uma das barreiras do circuito e verifique se o robô as atinge.

3. Agora, você será direcionado para um ambiente onde não estará fisicamente próximo ao robô. Tente movê-lo de onde ele está até o fim do circuito e retorne ao ponto de partida.

4. Retornando a uma posição próxima ao robô, tente acionar algo na interface que, em caso de emergência, pare completamente seu funcionamento.

## Resultado dos testes

&emsp;A seguir, é possível visualizar a tabela com os resultados

| Usuário          | Ocupação                          | Número do Desafio | Foi possível completar o desafio | Observação                                                                                                                                  | Sugestão do Usuário                                                                                         |
|------------------|-----------------------------------|-------------------|----------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------|
| Mateus Marçal   | Graduando em Sistemas de Informação| 01                | Sim                              | -                                                                                                                                           | Melhora na latência da câmera do robô, uma vez que foi difícil acompanhar algumas vezes                                                            |
|                  |                                   | 02                | Médio                            | Apesar de conseguir movimentar o robô em direção à barreira e ele parar com êxito, o pop up não foi acionado na interface                    |                                                                                                              |
|                  |                                   | 03                | Sim                              | -                                                                                                                                           |                                                                                                              |
|                  |                                   | 04                | Não                              | O botão, mesmo após pressionado, não interrompeu a movimentação do robô                                                                      |                                                                                                              |
| Victor Carvalho  | Graduando em Engenharia de Software| 01              | Sim                              | -                                                                                                                                           | Melhora na latência da câmera do robô, uma vez que foi difícil acompanhar algumas vezes                                                            |
|                  |                                   | 02                | Médio                            | Apesar de conseguir movimentar o robô em direção à barreira e ele parar com êxito, o pop up não foi acionado na interface                    |                                                                                                              |
|                  |                                   | 03                | Sim                              | -                                                                                                                                           |                                                                                                              |
|                  |                                   | 04                | Não                              | O botão, mesmo após pressionado, não interrompeu a movimentação do robô                                                                      |                                                                                                              |
| Willians Pantaleão| Bibliotecário                     | 01                | Sim                              | A movimentação do robô por meio do teclado não foi tão intuitiva, em um primeiro momento, tentou clicar nos botões da interface por meio do mouse. | A posição do botão de parada de emergência ficou muito afastado do contexto da teleoperação, e não muito intuitivo                                                           |
|                  |                                   | 02                | Médio                            | Apesar de conseguir movimentar o robô em direção à barreira e ele parar com êxito, o pop up não foi acionado na interface                    |                                                                                                              |
|                  |                                   | 03                | Médio                            | Por conta da alta taxa de latência, a teleoperação por meio da câmera foi comprometida. Porém, estabilizou depois e foi possível finalizar o desafio |                                                                                                              |
|                  |                                   | 04                | Não                              | O botão, mesmo após pressionado, não interrompeu a movimentação do robô                                                                      |                                                                                                             
| Karoline Nardo   | Parceira da Atvos                 | 01                | Sim                              | -                                                                                                                                           | A posição do botão de parada de emergência poderia estar em maior destaque na interface                                                                                                 |
|                  |                                   | 02                | Médio                            | Apesar de conseguir movimentar o robô em direção à barreira e ele parar com êxito, o pop up não foi acionado na interface                    |                                                                                                              |
|                  |                                   | 03                | Não                              | A latência estava muito alta, sendo assim, a transmissão estava com grande delay, o que não tornou possível a movimentação apenas pela câmera |                                                                                                              |
|                  |                                   | 04                | Não                              | O botão, mesmo após pressionado, não interrompeu a movimentação do robô                                                                      |                                                                                                                                      
 ## Melhorias e Pontos de Observação

&emsp;Com base nos testes realizados com os usuários e no feedback obtido, tanto por meio de sugestões quanto por observações dos comportamentos dos usuários durante a interação com a interface, é importante ressaltar as seguintes melhorias:

**Melhoria na Latência** - Alto Grau de Prioridade

&emsp;A latência da transmissão dos dados da câmera é um ponto crítico, sendo apresentada como um problema recorrente durante os testes. Isso impacta diretamente a funcionalidade do sistema, impedindo a conclusão bem-sucedida do terceiro caso de teste e a operação eficiente do robô de forma teleoperada.

**Funcionamento do Botão de Emergência** - Alto Grau de Prioridade

&emsp;O botão de emergência apresentou problemas de funcionamento de forma recorrente nos testes, demonstrando que o segundo caso de teste não foi concluído com êxito. Se não for corrigido, isso impedirá a parada total do robô em situações de emergência.

**Pop-up de Aviso ao Encontrar Impedimento** - Médio Grau de Prioridade

&emsp;Embora o não acionamento do pop-up seja um problema para o feedback do usuário, pois pode ser interpretado como um erro de funcionamento ao invés de uma parada proposital para evitar colisões, a funcionalidade principal foi concluída com sucesso.

**Posição do Botão de Emergência** - Baixo Grau de Prioridade

&emsp;Em dois casos foi mencionado que a posição do botão de emergência não era evidente na interface, o que pode ser um problema. No entanto, se sua função principal for garantida, essa melhoria possui baixo grau de prioridade. Ainda assim, o local do botão pode ser ajustado para ficar mais confortável para o usuário.

&emsp;A análise dos testes e o feedback dos usuários revelaram pontos críticos que precisam ser abordados para melhorar a usabilidade e a segurança do sistema. A latência na transmissão de dados e o funcionamento do botão de emergência são as prioridades mais altas e necessitam de atenção imediata. Melhorias de médio e baixo impacto, como o acionamento do pop-up de aviso e a posição do botão de emergência, também devem ser consideradas para aprimorar a experiência do usuário. Implementar essas melhorias garantirá um sistema mais eficiente, seguro e intuitivo.