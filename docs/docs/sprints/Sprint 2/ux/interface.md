---
title: "Protótipo da Interface de Usuário"
sidebar_position: 1
---

&emsp;Durante a Sprint 2, o grupo Cannabot deu início ao processo de desenvolvimento de um ambiente dedicado ao acompanhamento e controle dos dados coletados pela solução proposta. Esta etapa visa possibilitar aos usuários monitorar o desempenho do robô, além de oferecer a capacidade de controle remoto, quando necessário.


&emsp;Com base nas user stories e personas definidas durante a Sprint 2, identificaram-se as necessidades dos usuários e os elementos de funcionamento essenciais que devem estar inseridas na interface a ser desenvolvida. Para facilitar a visualização e entendimento desses elementos, optou-se por criar inicialmente o Wireframe, que é uma representação esquemática e simplificada da interface, destacando sua estrutura, funcionalidades e disposição dos elementos. Em seguida, foi elaborado o Mockup - uma representação mais fiel e detalhada da interface, incluindo elementos visuais como cores, tipografia e imagens, proporcionando uma prévia mais realista do produto final.

Esse desenvolvimento pode ser acompanhado a seguir:

## Wireframe

&emsp;A seguir, é possível acompanhar o fluxo das telas e suas funcionalidades

&emsp;O início da interface priorizou o acesso à plataforma. Com essa premissa, foram concebidas as telas de cadastro e login do usuário.

<div align="center"> 

![Tela de Cadastro - Wireframe](../../../../..\docs\static\img\sprint2\wireframe\cadastrowir.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

<div align="center"> 

![Tela de Login - Wireframe](../../../../..\docs\static\img\sprint2\wireframe\loginwir.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

&emsp;Após o login, o usuário é redirecionado para a tela inicial (Histórico de Dados) da plataforma, que disponibiliza o último heatmap (mapa de calor) gerado pela coleta de dados do robô, ao lado, têm-se uma tabela com quatro colunas sendo elas: data, hora, localização e temperatura, informações coletadas e salvas pela solução. Ademais, a interface possui um menu lateral persistente em todas as telas do site, facilitando a navegação sem a necessidade de retornar à tela inicial repetidamente.

<div align="center"> 

![Histórico de Dados - Wireframe](../../../../..\docs\static\img\sprint2\wireframe\historicowir.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

<div align="center"> 

![Histórico de Dados e Menu - Wireframe](../../../../..\docs\static\img\sprint2\wireframe\historicomenuwir.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Ao selecionar "Central de Controle", o usuário é conduzido à tela de acompanhamento do desempenho do robô no modo autônomo. Nessa tela, é possível visualizar a transmissão ao vivo da imagem capturada pelo robô, juntamente com a temperatura registrada pelo seu sensor. Ademais, há o botão de "Modo de emergência", permitindo a interrupção imediata da operação do robô.

<div align="center"> 

![Central de Controle Automática - Wireframe](../../../../..\docs\static\img\sprint2\wireframe\controleautowir.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Para alternar entre o modo autônomo e o modo manual (teleoperado), basta ao usuário acionar o switch, direcionando-o para outra tela com as mesmas funcionalidades, acrescidas do controle remoto para manipular o movimento do robô.

<div align="center"> 

![Central de Controle Manual - Wireframe](../../../../..\docs\static\img\sprint2\wireframe\controlemanwir.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Ao clicar em "Registros de Uso", o usuário é encaminhado a uma tela que apresenta uma tabela com informações sobre usuário, data e hora de acesso à plataforma, proporcionando um controle eficiente do processo.

<div align="center"> 

![Central de Controle Manual - Wireframe](../../../../..\docs\static\img\sprint2\wireframe\registrowir.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

E por fim, ao clicar em "Histórico de Dados", o usuário retorna para à página inicial.

<div align="center"> 

![Histórico de Dados - Wireframe](../../../../..\docs\static\img\sprint2\wireframe\historicowir.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

## Mockup

Depois de elaborado o wireframe, o grupo prototipou o Mockup de alta fidelidade. O fluxo se inicia pela tela de cadastro, caso seja um novo usuário:

<div align="center"> 

![Tela de Cadastro - Mockup](../../../../..\docs\static\img\sprint2\mockup\cadastromoc.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Ou através da tela de login se já for um usuário cadastrado:

<div align="center"> 

![Tela de Login - Mockup](../../../../..\docs\static\img\sprint2\mockup\loginmoc.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Então, o fluxo prossegue para a tela de home, que exibe o histórico de dados:

<div align="center"> 

![Histórico de Dados - Mockup](../../../../..\docs\static\img\sprint2\mockup\historicomoc.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Todas as telas também acompanham a seguinte sidebar:

<div align="center"> 

![Sidebar - Mockup](../../../../..\docs\static\img\sprint2\mockup\historicosidemoc.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Usando a sidebar para navegar até a central de controle, a seguinte tela - Central de Controle Autônomo - é exibida:

<div align="center"> 

![Central de Controle Autonômo - Mockup](../../../../..\docs\static\img\sprint2\mockup\controleautomoc.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Se utilizado a função de desativar o modo autônomo, a visualização muda para:

<div align="center"> 

![Central de Controle Manual - Mockup](../../../../..\docs\static\img\sprint2\mockup\controlemanumoc.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

Ainda através da sidebar, o usuário pode navegar para os registros de uso:

<div align="center"> 

![Registros de Uso - Mockup](../../../../..\docs\static\img\sprint2\mockup\registromoc.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

E novamente, o fluxo acaba na tela de histórico de dados (home): 

<div align="center"> 

![Histórico de Dados - Mockup](../../../../..\docs\static\img\sprint2\mockup\historicomoc.png)
Fonte: Elaborado pelo grupo Cannabot
</div>