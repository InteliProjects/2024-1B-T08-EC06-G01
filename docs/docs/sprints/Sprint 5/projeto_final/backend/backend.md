---
title: "Documentação das rotas e schemas do projeto"
sidebar_position: 3
description: Nessa secção iremos explicar como realizamos a construção das rotas e dos schemas do projeto versão 3.0.
---

# Documentação do Backend

## Introdução

&emsp;Esta seção da documentação é dedicada a descrever as funcionalidades e o funcionamento do backend do nosso projeto de teleoperação com ROS2. O backend é responsável por gerenciar as operações de registro, consulta, atualização e exclusão em cada diferente tabela do banco de dados que registram as ações realizadas durante a teleoperação. Esta documentação é essencial para desenvolvedores que desejam compreender o fluxo de dados e a estrutura das rotas da API, facilitando a manutenção e futuras implementações no sistema.

## Roteamento do backend

### Arquivo routes.py

&emsp;O arquivo `routes.py` é a espinha dorsal do sistema de roteamento do nosso backend. Ele centraliza e organiza todas as rotas da aplicação, garantindo que cada módulo específico de funcionalidade (como usuários, logs, mídias, robôs, websockets, temperatura e localização) tenha seu próprio roteador, o que facilita a manutenção e a escalabilidade do código.

### Estrutura e Funcionalidade:

```python
from fastapi import APIRouter
from routes.logs import router as logs_router
from routes.medias import router as medias_router
from routes.robots import router as robots_router
from routes.users import router as users_router
from routes.websocket import router as websocket_router
from routes.temp import router as temp_router

router = APIRouter()

router.include_router(temp_router)
router.include_router(users_router)
router.include_router(logs_router)
router.include_router(medias_router)
router.include_router(robots_router)
router.include_router(websocket_router)
```

&emsp;Para conectar essas as rotas de cada schema ao nosso arquivo `routes.py` usamos esse trecho em especifico de código em cada arquivo da pasta `routes` que define o prefixo e tags para rotas, aqui como exemplo estamos utilizando o prefixo de logs:

```python
router = APIRouter(
    prefix="/logs",
    tags=["logs"],
)
```

&emsp;Ao organizar as rotas dessa maneira, conseguimos modularizar nosso código, o que melhora a clareza e a manutenção. Cada módulo pode ser desenvolvido e testado separadamente, e qualquer mudança ou adição de novas rotas pode ser feita sem impactar diretamente outras partes da aplicação. Além disso, o uso de APIRouter do FastAPI nos permite criar um sistema de roteamento robusto e eficiente, adequado para aplicações de médio e grande porte.

&emsp;A abordagem utilizada no routes.py oferece uma estrutura clara e escalável para o gerenciamento das rotas do backend, facilitando tanto o desenvolvimento quanto a manutenção do sistema. Com esta organização, novos desenvolvedores podem facilmente entender e contribuir para o projeto, assegurando que o fluxo de dados e as operações do backend sejam eficientes e bem estruturadas.

## Schema e Route's da tabela log:

### Schema Log

&emsp;O schema logs define a estrutura dos dados armazenados na tabela logs e é utilizado para validação e serialização dos dados no FastAPI. Abaixo está o código do schema logs:

```python
from datetime import datetime
from typing import Optional
from datetime import datetime
from pydantic import BaseModel, Field
from datetime import datetime

class Log(BaseModel):
    id: int = Field(default=None, gt=0)
    date: Optional[datetime] = Field(default=None)
    emergency_button: bool = Field(default=False)
    ia_request: bool = Field(default=False)
    username: str = Field(default=None)
    user_id: int = Field(default=None)
```

- `id`: Identificador único do log. É um inteiro positivo.
- `date`: Data e hora em que a ação foi registrada. É opcional.
- `emergency_button`: Ação ao apertar o botão de emergência da aplicação. É um booleano.
- `ia_request`: Ação para ativar a IA da aplicação que verifica sujeira atrvés da visão computacional.
- `username`: Campo para registrar qual usuário fez uso da aplicação.
- `user_id`: Chave estrangeira que referencia o usuário que acessou  a aplicação.

### Rotas de Logs

&emsp;As rotas para a tabela `logs` permitem a listagem, consulta individual, atualização e exclusão de logs. Abaixo estão as rotas definidas no arquivo `routes/logs.py`:

```python
from datetime import datetime
from uuid import UUID

import ormar
from fastapi import HTTPException
from fastapi import APIRouter
from fastapi.responses import JSONResponse
from models.logs import Log as LogModel
from schemas.logs import Log
from ormar.exceptions import NoMatch


router = APIRouter(
    prefix="/logs",
    tags=["logs"],
)

@router.post("/register")
async def register(log: Log):
    try:
        await LogModel.objects.create(
            date = log.date,
            emergency_button = log.emergency_button,
            ia_request = log.ia_request,
            user_id = log.user_id
        )
        return JSONResponse(content={
            "error": False,
            "message": "Log criado com sucesso"
        }, status_code=201)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/list")
async def list_logs():
    try:
        logs = await LogModel.objects.all()
        if not logs:
            return JSONResponse(content={
                "error": True,
                "message": "Nenhum log encontrado"
            }, status_code=404)
        
        log_dicts = []
        for log in logs:
            log_dict = log.dict()
            for key, value in log_dict.items():
                if isinstance(value, datetime):
                    log_dict[key] = value.isoformat()
            log_dicts.append(log_dict)

        return JSONResponse(content={
            "error": False,
            "message": "Logs encontrados com sucesso",
            "data": log_dicts
        }, status_code=200)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/list/{log_id}")
async def list(log_id: int):
    try:
        log = await LogModel.objects.get(id=log_id)
        print(log)
        return JSONResponse(content={
            "error": False,
            "log": log
        }, status_code=200)
    except ormar.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Log não encontrado"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.put("/update/{log_id}")
async def update(log_id: int, log_update: Log):
    try:
        existing_log = await LogModel.objects.get_or_none(id=log_id)
        if existing_log is None:
            raise HTTPException(status_code=404, detail="Log não encontrado")

        log_update_data = log_update.dict(exclude_unset=True)
        for field, value in log_update_data.items():
            if isinstance(value, datetime):
                log_update_data[field] = value.isoformat()

        await existing_log.update(**log_update_data)

        return {"error": False, "message": "Log atualizado com sucesso"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro interno do servidor: {e}")



@router.delete("/delete/{log_id}")
async def delete(log_id: int):
    try:
        await LogModel.objects.delete(id=log_id)
        return JSONResponse(content={
            "error": False,
            "message": "Log deletado com sucesso"
        }, status_code=200)
    except ormar.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Log não encontrado"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)
```

&emsp;Explicação um pouco mais detalhada de cada rota em específico:

:::warning

Movemos a rota **POST** de `log` para o arquivo de rotas de `user`. Dessa forma, quando um usuário fizer login em nossa aplicação, a rota de login automaticamente chama `POST-log`, criando um registro que está atrelado ao usuário. Para mais detalhes, consulte a documentação das rotas da tabela `user`.

:::

- *`POST /register`:
    Cria um novo log com os dados fornecidos.
    Retorna sucesso (201) ou erro (500).

- `GET /list`:
    Lista todos os logs.
    Formata os dados antes de retornar.
    Retorna sucesso (200) com logs ou erro (404/500).

- `GET /list/{log_id}`:
    Consulta um log específico pelo ID.
    Formata os dados antes de retornar.
    Retorna sucesso (200) com o log ou erro (404/500).

- `PUT /update/{log_id}`:
    Consulta um log específico pelo ID.
    Atualiza um log existente com os dados fornecidos.
    Retorna sucesso (200) ou erro (500).

- `DELETE /delete/{log_id}`:
    Deleta um log específico pelo ID.
    Retorna sucesso (200) ou erro (404/500).

## Schema e Route's da tabela media:

### Schema Media

&emsp;O schema `media` define a estrutura dos dados armazenados na tabela `media` e é utilizado para validação e serialização dos dados no FastAPI. Abaixo está o código do schema `media`:

```python
from datetime import datetime
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, Field

class Media(BaseModel):
    uuid: Optional[UUID] = Field(default=None)
    title: str = Field(default=None, max_length=100)
    type: bool = Field(default=False)
    date: Optional[datetime] = Field(default=None)
    robot_id: Optional[int] = Field(default=None)
```
- `uuid`: Identificador único da mídia. É um UUID opcional.
- `title`: Título da mídia. É uma string com no máximo 100 caracteres.
- `type`: Tipo de mídia. É um booleano.
- `date`: Data e hora em que a mídia foi registrada. É opcional.
- robot_id: ID do robô associado à mídia. É um inteiro opcional.

### Rotas de Media

&emsp;As rotas para a tabela `media` permitem a criação, listagem, consulta individual, atualização e exclusão de mídias. Abaixo estão as rotas definidas no arquivo `routes/medias.py`:

```python
import uuid
from datetime import datetime

import ormar
from fastapi import APIRouter
from fastapi.responses import JSONResponse
from models.medias import Media as MediaModel
from schemas.medias import Media

router = APIRouter(
    prefix="/medias",
    tags=["medias"],
)

@router.post("/register")
async def register(media: Media):
    try:
        await MediaModel.objects.create(
            title=media.title,
            type=media.type,
            date=datetime.now(),
            robot_id=media.robot_id,
        )
        return JSONResponse(content={
            "error": False,
            "message": "Media criada com sucesso"
        }, status_code=201)
    except Exception as e:
        if "UNIQUE constraint failed" in str(e):
            return JSONResponse(content={
                "error": True,
                "message": "Problema ao criar mídia: já existe uma media com essa primary key"
            }, status_code=400)
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/list")
async def list():
    try:
        medias = await MediaModel.objects.all()
        if not medias:
            return JSONResponse(content={
                "error": True,
                "message": "Nenhuma mídia encontrada"
            }, status_code=404)

        media_dicts = []
        for media in medias:
            media_dict = media.dict()
            for key, value in media_dict.items():
                if isinstance(value, uuid.UUID):
                    media_dict[key] = str(value)
                elif isinstance(value, datetime):
                    media_dict[key] = value.isoformat()
            media_dicts.append(media_dict)

        return JSONResponse(content={
            "error": False,
            "message": "Medias listadas com sucesso",
            "data": media_dicts
        }, status_code=200)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/get/{media_id}")
async def get(media_id: uuid.UUID):
    try:
        media = await MediaModel.objects.get(uuid=media_id)

        media_dict = media.dict()
        for key, value in media_dict.items():
            if isinstance(value, uuid.UUID):
                media_dict[key] = str(value)
            elif isinstance(value, datetime):
                media_dict[key] = value.isoformat()

        return JSONResponse(content={
            "error": False,
            "message": "Mídia encontrada",
            "data": media_dict
        }, status_code=200)
    except ormar.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Mídia não encontrada"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.put("/update")
async def update(media: Media):
    try:
        await MediaModel.objects.filter(uuid=media.uuid).update(
            title=media.title,
            type=media.type,
            robot_id=media.robot_id
        )
        return JSONResponse(content={
            "error": False,
            "message": "Mídia atualizada"
        }, status_code=200)
    except ormar.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Mídia não encontrada"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.delete("/delete/{media_id}")
async def delete(media_id: uuid.UUID):
    try:
        await MediaModel.objects.delete(uuid=media_id)
        return JSONResponse(content={
            "error": False,
            "message": "Mídia deletada"
        }, status_code=200)
    except ormar.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Mídia não encontrada"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)
```

&emsp;Explicação um pouco mais detalhada de cada rota em específico:

- `POST /register`:
    Cria uma nova mídia com os dados fornecidos.
    Retorna sucesso (201) ou erro (500).

- `GET /list`:
    Lista todas as mídias.
    Formata os dados antes de retornar.
    Retorna sucesso (200) com mídias ou erro (404/500).

- `GET /get/{media_id}`:
    Consulta uma mídia específica pelo ID (UUID).
    Formata os dados antes de retornar.
    Retorna sucesso (200) com a mídia ou erro (404/500).

- `PUT /update`:
    Atualiza uma mídia existente com os dados fornecidos.
    Retorna sucesso (200) ou erro (500).

- `DELETE /delete/{media_id}`:
    Deleta uma mídia específica pelo ID (UUID).
    Retorna sucesso (200) ou erro (404/500).

## Schema e Route's da tabela robot:
### Schema Robot

&emsp;O schema robot define a estrutura dos dados armazenados na tabela robots e é utilizado para validação e serialização dos dados no FastAPI. Abaixo está o código do schema robot:

```python
from pydantic import BaseModel, Field

class Robot(BaseModel):
    id: int = Field(default=None, gt=0)
    name: str = Field(default=None, max_length=100)
    user_id: int = Field(default=None)
```

- `id`: Identificador único do robô. É um inteiro positivo.
- `name`: Nome do robô. É uma string com no máximo 100 caracteres.
- `user_id`: ID do usuário associado ao robô. É um inteiro.

### Rotas de Robot

 As rotas para a tabela robots permitem a criação, listagem, consulta individual, atualização e exclusão de robôs. Abaixo estão as rotas definidas no arquivo routes/robots.py:

```python
import ormar
import ormar.exceptions
from fastapi import APIRouter
from fastapi.responses import JSONResponse
from models.robots import Robot as RobotModel
from.schemas.robots import Robot

router = APIRouter(
    prefix="/robots",
    tags=["robots"],
)

@router.post("/register")
async def register(robot: Robot):
    try:
        await RobotModel.objects.create(
            name=robot.name,
            user_id=robot.user_id
        )
        return JSONResponse(content={
            "error": False,
            "message": "Robot criado com sucesso"
        }, status_code=201)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/list")
async def list():
    try:
        robots = await RobotModel.objects.all()
        return JSONResponse(content={
            "error": False,
            "message": "Robots encontrados com sucesso",
            "data": robots
        }, status_code=200)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/get/{robot_id}")
async def get(robot_id: int):
    try:
        robot = await RobotModel.objects.get(id=robot_id)
        return JSONResponse(content={
            "error": False,
            "message": "Robot encontrado com sucesso",
            "data": robot
        }, status_code=200)
    except ormar.exceptions.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Robot não encontrado"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.put("/update")
async def update(robot: Robot):
    try:
        await RobotModel.objects.filter(id=robot.id).update(
            name=robot.name,
            user_id=robot.user_id
        )
        return JSONResponse(content={
            "error": False,
            "message": "Robot atualizado com sucesso"
        }, status_code=200)
    except ormar.exceptions.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Robot não encontrado"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.delete("/delete/{robot_id}")
async def delete(robot_id: int):
    try:
        await RobotModel.objects.delete(id=robot_id)
        return JSONResponse(content={
            "error": False,
            "message": "Robot deletado com sucesso"
        }, status_code=200)
    except ormar.exceptions.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Robot não encontrado"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)
```

&emsp;Explicação um pouco mais detalhada de cada rota em específico:

- `POST /register`:
    Cria um novo robô com os dados fornecidos.
    Retorna sucesso (201) ou erro (500).

- `GET /list`:
    Lista todos os robôs.
    Retorna sucesso (200) com robôs ou erro (500).

- `GET /get/{robot_id}`:
    Consulta um robô específico pelo ID.
    Retorna sucesso (200) com o robô ou erro (404/500).

- `PUT /update`:
    Atualiza um robô existente com os dados fornecidos.
    Retorna sucesso (200) ou erro (404/500).

- `DELETE /delete/{robot_id}`:
    Deleta um robô específico pelo ID.
    Retorna sucesso (200) ou erro (404/500).

## Schema e Route's da tabela user:
### Schema User

&emsp;O schema user define a estrutura dos dados armazenados na tabela users e é utilizado para validação e serialização dos dados no FastAPI. Abaixo está o código do schema user:

```python
from pydantic import BaseModel, Field

class User(BaseModel):
    id: int = Field(default=None, gt=0)
    username: str = Field(default=None, max_length=100)
    password: str = Field(default=None, max_length=100)
    admin: bool = Field(default=False)
```

- `id`: Identificador único do usuário. É um inteiro positivo.
- `username`: Nome de usuário. É uma string com no máximo 100 caracteres.
- `password`: Senha do usuário. É uma string com no máximo 100 caracteres.
- `admin`: Indica se o usuário é administrador. É um booleano.

### Rotas de User

&emsp;As rotas para a tabela `users` permitem a criação, listagem, consulta individual, atualização e exclusão de usuários. Abaixo estão as rotas definidas no arquivo `routes/users.py`:

```python
import ormar
import ormar.exceptions
from fastapi import APIRouter
from fastapi.responses import JSONResponse
from models.users import User as UserModel
from schemas.users import User
from utils.crypto import get_password_hash, verify_password
from models.logs import Log as LogModel
from schemas.logs import Log
from pytz import timezone
from datetime import datetime

router = APIRouter(
	prefix="/users",
	tags=["users"],
)

@router.post("/register")
async def register(user: User):
    try:
        existing_user = await UserModel.objects.get_or_none(username=user.username)
        if existing_user:
            return JSONResponse(content={
                "error": True,
                "message": "Usuário já existe"
            }, status_code=400)

        await UserModel.objects.create(
            username=user.username,
            password=get_password_hash(user.password),
            admin=False
        )
        return JSONResponse(content={
            "error": False,
            "message": "Usuário criado com sucesso"
        }, status_code=201)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)


@router.post("/login")
async def login(user: User):
	try:
		result = await UserModel.objects.get(username=user.username)
		print(result)
		print(result.password)
		if verify_password(user.password, result.password):
			print("Senha válida")	
			print(result.id, result.username)
			await create_log(result.id, result.username)
			return result
		else:
			return {"error": "Invalid password"}
	except ormar.exceptions.NoMatch:
		print("Usuário não encontrado")
		return {"error": "User not found"}
	except Exception as e:
		return JSONResponse(content={
			"error": True,
			"message": f"Erro interno do servidor: {e}"
		}, status_code=500)
	
async def create_log(user_id, username):
    print("Criando log")
    print(user_id, username)
    try:
        print("entrei no try")
        
        current_time = datetime.now(tz=timezone('America/Sao_Paulo'))
        print(current_time)
        current_time_naive = current_time.replace(tzinfo=None)

        await LogModel.objects.create(
            date=current_time_naive,
            emergency_button=False,
            ia_request=False,
            user_id=user_id,
            username=username
        )
        print("Log criado com sucesso")
    except Exception as e:
        print(f"Erro ao criar log: {e}")
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)



@router.get("/list")
async def list():
    try:
        users_all = await UserModel.objects.all()
        users_list = [user.dict() for user in users_all]
        return JSONResponse(content={
            "error": False,
            "message": "Usuários encontrados com sucesso",
            "data": users_list
        }, status_code=200)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/get/{user_id}")
async def get(user_id: int):
	try:
		user = await UserModel.objects.get(id=user_id)
		user_dict = user.dict()
		return JSONResponse(content={
			"error": False,
			"message": "Usuário encontrado com sucesso",
			"data": user_dict
		}, status_code=200)
	except ormar.exceptions.NoMatch:
		return JSONResponse(content={
			"error": True,
			"message": "Usuário não encontrado"
		}, status_code=404)
	except Exception as e:
		return JSONResponse(content={
			"error": True,
			"message": f"Erro interno do servidor: {e}"
		}, status_code=500)

@router.put("/update")
async def update(user: User):
	try:
		await UserModel.objects.filter(id=user.id).update(
			username=user.username,
			password=get_password_hash(user.password),
			admin=user.admin
		)
		return JSONResponse(content={
			"error": False,
			"message": "Usuário atualizado com sucesso"
		}, status_code=200)
	except ormar.exceptions.NoMatch:
		return JSONResponse(content={
			"error": True,
			"message": "Usuário não encontrado"
		}, status_code=404)
	except Exception as e:
		return JSONResponse(content={
			"error": True,
			"message": f"Erro interno do servidor: {e}"
		}, status_code=500)

@router.delete("/delete/{user_id}")
async def delete(user_id: int):
	try:
		await UserModel.objects.delete(id=user_id)
		return JSONResponse(content={
			"error": False,
			"message": "Usuário deletado com sucesso"
		}, status_code=200)
	except ormar.exceptions.NoMatch:
		return JSONResponse(content={
			"error": True,
			"message": "Usuário não encontrado"
		}, status_code=404)
	except Exception as e:
		return JSONResponse(content={
			"error": True,
			"message": f"Erro interno do servidor: {e}"
		}, status_code=500)
```

&emsp;Explicação um pouco mais detalhada de cada rota em específico:

- `POST /register`:
    Cria um novo usuário com os dados fornecidos.
    Retorna sucesso (201) ou erro (500).

- *`POST /login`:
    Realiza login do usuário com base nas credenciais fornecidas e chama a função `create_log()` que cria um novo log na aplicação com os dados do usuário que está realizando o login.
    Retorna sucesso (200) com os dados do usuário ou erro (404/500).

- `GET /list`:
    Lista todos os usuários.
    Retorna sucesso (200) com a lista de usuários ou erro (500).

- `GET /get/{user_id}`:
    Consulta um usuário específico pelo ID.
    Retorna sucesso (200) com os dados do usuário ou erro (404/500).

- `PUT /update`:
    Atualiza um usuário existente com os dados fornecidos.
    Retorna sucesso (200) ou erro (404/500).

- `DELETE /delete/{user_id}`:
    Deleta um usuário específico pelo ID.
    Retorna sucesso (200) ou erro (404/500).

## Schema e Route's da tabela temp:
### Schema Temp

&emsp;O schema temp define a estrutura dos dados armazenados na tabela temp e é utilizado para validação e serialização dos dados no FastAPI. Abaixo está o código do schema temp:
>**Nota**: temp é a referência utilizada para a palavra `temperatura`.

```python
from pydantic import BaseModel, Field

class Temp(BaseModel):
	id: int = Field(default=None, gt=0)
	temp: float = Field(default=None)
	robot_id: int = Field(default=None)
```

- `id`: Identificador único da temperatura. É um inteiro positivo.
- `temp`: Aqui nesse campo é armazenado o valor capturado pelo sensor de temperatura do robô no momento da leitura.
- `robot_id`: Identificador estrangeiro da tabela robot, utilizado para referenciar o robô que fez a leitura da temperatura.

### Rotas de Temp

&emsp;As rotas para a tabela `temp` permitem a criação, listagem, consulta individual, atualização e exclusão de temperaturas capturadas por um robô. Abaixo estão as rotas definidas no arquivo `routes/temp.py`:

```python
import ormar
import ormar.exceptions
from fastapi import APIRouter
from fastapi.responses import JSONResponse
from models.temp import Temp as TempModel
from schemas.temp import Temp
from datetime import datetime
from pytz import timezone
import numpy as np
import matplotlib.pyplot as plt
import base64
from io import BytesIO

router = APIRouter(
	prefix="/temp",
	tags=["temp"],
)

@router.post("/register")
async def register(temp: Temp):
    try:
        current_time = datetime.now(tz=timezone('America/Sao_Paulo'))
        current_time_naive = current_time.replace(tzinfo=None)

        await TempModel.objects.create(
            temp=temp.temp,
            robot_id=temp.robot_id,
            location_x=temp.location_x,
            location_y=temp.location_y,
            date=current_time_naive,
        )
        return JSONResponse(content={
            "error": False,
            "message": "Temp criado com sucesso"
        }, status_code=201)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/list")
async def list():
    try:
        temp = await TempModel.objects.all()
        if not temp:
            return JSONResponse(content={
                "error": True,
                "message": "Nenhum temp encontrado"
            }, status_code=404)
        
        temp_dicts = []
        data_for_heatmap = []  
        for t in temp:
            temp_dict = t.dict()
            for key, value in temp_dict.items():
                if isinstance(value, float):
                    temp_dict[key] = float(value)
                if isinstance(value, datetime):
                    temp_dict[key] = value.isoformat()
            temp_dicts.append(temp_dict)
    
            x = temp_dict.get('location_x')
            y = temp_dict.get('location_y')
    
            if x is not None and y is not None:
                data_for_heatmap.append((x, y, temp_dict['temp']))
    
        img_base64 = generate_heatmap(data_for_heatmap) if data_for_heatmap else None
    
        return JSONResponse(content={
            "error": False,
            "message": "Temp encontrados com sucesso",
            "data": temp_dicts,
            "heatmap": img_base64
        }, status_code=200)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.get("/get/{temp_id}")
async def get(temp_id: int):
    try: 
        temp = await TempModel.objects.get(id=temp_id)

        if not temp:
            return JSONResponse(content={
                "error": True,
                "message": "Temp não encontrado"
            }, status_code=404)
        
        temp_dict = temp.dict()
        for key, value in temp_dict.items():
            if isinstance(value, float):
                temp_dict[key] = float(value)
            if isinstance(value, datetime):
                temp_dict[key] = value.isoformat()
        
        return JSONResponse(content={
            "error": False,
            "message": "Temp encontrado com sucesso",
            "temp": temp_dict
        }, status_code=200)
    except ormar.exceptions.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Temp não encontrado"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.put("/update/{temp_id}")
async def update(temp_id: int, temp: Temp):
    try:
        existing_temp = await TempModel.objects.get(id=temp_id)

        update_data = temp.dict(exclude_unset=True)

        if 'date' in update_data:
            del update_data['date']

        for key, value in update_data.items():
            setattr(existing_temp, key, value)

        await existing_temp.update()

        return JSONResponse(content={
            "error": False,
            "message": "Temp atualizado com sucesso"
        }, status_code=200)
    except ormar.exceptions.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Temp não encontrado"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)

@router.delete("/delete/{temp_id}")
async def delete(temp_id: int):
    try:
        await TempModel.objects.delete(id=temp_id)
        return JSONResponse(content={
            "error": False,
            "message": "Temp deletado com sucesso"
        }, status_code=200)
    except ormar.exceptions.NoMatch:
        return JSONResponse(content={
            "error": True,
            "message": "Temp não encontrado"
        }, status_code=404)
    except Exception as e:
        return JSONResponse(content={
            "error": True,
            "message": f"Erro interno do servidor: {e}"
        }, status_code=500)
```

&emsp;Explicação um pouco mais detalhada de cada rota em específico:

- `POST /register`:
    Cria uma nova temperatura e sincroniza a localização com os dados fornecidos afim de gerar o heatmap.
    Retorna sucesso (201) ou erro (500).

- `GET /list`:
    Lista todas as temperaturas e as localizações, de todos os robôs, armazenadas no banco.
    Retorna sucesso (200) com a lista de usuários ou erro (500).

- `GET /get/{temp_id}`:
    Consulta uma temperatura e localização específicas pelo ID.
    Retorna sucesso (200) com os dados da tabela temperatura ou erro (404/500).

- `PUT /update/{temp_id}`:
    Consulta uma temperatura e localização específicas pelo ID.
    Atualiza um registro de temperatura e localização existente com os dados fornecidos.
    Retorna sucesso (200) ou erro (404/500).

- `DELETE /delete/{temp_id}`:
    Deleta um registro de temperatura e localização específicos pelo ID.
    Retorna sucesso (200) ou erro (404/500).


## Rota de WebSocket

&emsp;As rotas para o WebSocket permitem a comunicação em tempo real entre o frontend e o backend. Abaixo estão as rotas definidas no arquivo `routes/websocket.py`:

```python
import json

import pydantic
from client.robot import robot
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from schemas.websocket import ControlPacket
from websockets.exceptions import ConnectionClosedError

router = APIRouter(
	prefix="/websocket",
	tags=["websocket"],
)

@router.websocket("/control")
async def control_robot(websocket: WebSocket):
	await websocket.accept()

	await robot.add_client(websocket)

	try:
		while True:
			raw = await websocket.receive_text()

			if not robot.websocket:
				continue

			try: data = json.loads(raw)
			except json.JSONDecodeError:
				await websocket.send_json({ "type": "SPacketError", "data": {
					"message": "JSON inválido"
				}})
				continue

			try: packet = ControlPacket(**data)
			except pydantic.ValidationError as e:
				await websocket.send_json({ "type": "SPacketError", "data": {
					"message": str(e).replace('\n', '')
				}})
				continue

			await robot.send(json.dumps({ 'control': packet.data.state }))

	except WebSocketDisconnect:
		await robot.remove_client(websocket)
		await websocket.close()
	except ConnectionClosedError:
		print("Connection to robot has been lost.")
		await robot.reconnect()
		await websocket.send_json({ "type": "SPacketError", "data": {
			"message": "Conexão com o robô foi perdida"
		}})
		return
```

&emsp;Explicação um pouco mais detalhada da rota:

- `GET /control`:
    Estabelece uma conexão WebSocket com o frontend.
    Recebe e envia pacotes de controle para o robô.
    Trata erros de conexão e validação de pacotes.

&emsp;Essa rota é responsável por estabelecer uma conexão WebSocket com o frontend, permitindo a comunicação em tempo real entre o usuário e o robô. Ela recebe pacotes de controle do frontend, valida os dados recebidos e envia os comandos para o robô. Além disso, trata erros de conexão e validação de pacotes, garantindo uma comunicação eficiente e segura entre os sistemas.

&emsp;Explicação do código:

- `await websocket.accept()`: Aceita a conexão WebSocket com o frontend.
- `await robot.add_client(websocket)`: Adiciona o cliente (frontend) à lista de clientes do robô. Isso garante que os pacotes enviados pelo robô sejam visualizados pelo frontend.
- `while True:`: Loop infinito para receber e enviar pacotes de controle.
- `raw = await websocket.receive_text()`: Recebe um pacote de controle do frontend.
- `try: data = json.loads(raw)`: Converte o pacote de controle em um objeto JSON.
- `try: packet = ControlPacket(**data)`: Valida o pacote de controle com o schema ControlPacket.
- `await robot.send(json.dumps({ 'control': packet.data.state }))`: Envia o comando de controle para o robô.
- `except WebSocketDisconnect:`: Trata a desconexão do frontend.
- `await robot.remove_client(websocket)`: Remove o cliente (frontend) da lista de clientes do robô.
- `await websocket.close()`: Fecha a conexão WebSocket.
- `except ConnectionClosedError:`: Trata a perda de conexão com o robô.
- `await robot.reconnect()`: Reconecta o backend ao robô.
- `await websocket.send_json({ "type": "SPacketError", "data": { "message": "Conexão com o robô foi perdida" }})`: Envia uma mensagem de erro ao frontend.

### Explicação da classe "robot" (client):

```python

class Robot:
	def __init__(self):
		self.websocket = None
		self.clients = set()

	async def connect(self):
		"""Connect to the robot WebSocket and start listening for messages."""
		print("Connecting to robot websocket...")
		self.websocket = await websockets.connect(ROBOT_WEBSOCKET_URL)
		asyncio.create_task(self._broadcast_forever())

	async def _broadcast_forever(self):
		"""Listen for messages from the WebSocket and broadcast them to all connected clients."""
		try:
			async for message in self.websocket: # type: ignore
				await self._broadcast(message)
		except ConnectionClosedError:
			print("Connection to robot has been lost, attempting to reconnect...")
			await self._broadcast(json.dumps({ "type": "SPacketError", "data": {
				"message": "Conexão com o robô foi perdida"
			}}))
			await self.reconnect()

	async def add_client(self, client: WebSocket):
		self.clients.add(client)
		print(f"Added client {client}")

	async def remove_client(self, client: WebSocket):
		self.clients.remove(client)
		print(f"Removed client {client}")

	async def send(self, data):
		"""Send a message to the robot."""
		if self.websocket is None:
			raise Exception("Not connected to robot")

		await self.websocket.send(data)

	async def _broadcast(self, message):
		if self.clients:
			await asyncio.gather(*[client.send_text(message) for client in self.clients])

	async def _reconnect(self):
		if self.websocket:
			try: await self.close()
			except: pass

		try: await self.connect()
		except Exception as e:
			print(f"Failed to reconnect to robot: {e}, retrying in 5 seconds...")
			await asyncio.sleep(5)
			return await self.reconnect()

		await self._broadcast(json.dumps({ "type": "SPacketInfo", "data": {
			"message": "Conexão com o robô estabelecida"
		}}))
		print("Connected to robot")

	async def reconnect(self):
		"""Close the current connection and reconnect to the robot WebSocket."""
		# use _reconnect in a non-blocking way
		asyncio.create_task(self._reconnect())

	async def close(self):
		if self.websocket:
			await self.websocket.close()
		self.websocket = None

robot = Robot()
```

&emsp;Essa classe é responsável por gerenciar a conexão com o robô via WebSocket, permitindo a comunicação em tempo real entre o backend e o robô. Ela estabelece a conexão com o robô, envia e recebe mensagens, e trata erros de conexão e desconexão. Além disso, ela mantém uma lista de clientes (frontend) conectados, para que os pacotes de controle enviados pelo robô sejam visualizados pelo frontend.

&emsp;Ela utiliza uma logica assíncrona para lidar com a conexão e desconexão do robô, garantindo que a comunicação seja eficiente e confiável. Além disso, ela envia mensagens de erro e informação para o frontend, permitindo que o usuário saiba o status da conexão com o robô. Com essa classe, é possível estabelecer uma comunicação bidirecional entre o frontend e o robô, facilitando o controle e monitoramento do robô durante a teleoperação e permitindo a futura adição de um sistema de autenticação e autorização para garantir a segurança da comunicação ao robô.

#### Formato do pacote de controle

```json
{
    "type": "CPacketControl",
    "data": {
        "state": <"forward" | "backward" | "left" | "right" | "stopped" | "emergency">
    }
}
```

&emsp;O pacote de controle é um objeto JSON que contém um campo `type` indicando o tipo de pacote e um campo `data` com os dados do pacote. O campo `state` contém a ação de controle a ser enviada para o robô, podendo ser "forward", "backward", "left", "right", "stopped" ou "emergency". Este formato é utilizado para padronizar a comunicação entre o frontend e o backend, facilitando o desenvolvimento e a manutenção do sistema.

### Explicação da classe "Camera" (client):

```python

class Camera:
    def __init__(self):
        self.websocket = None
        self.clients = set()

        # with open("yolo_v8_n_dirt_detection.pt", "wb") as f:
        #     print('oi eu sou o gustavo')
        self.yolo_model = YOLO("/app/client/yolo_v8_n_dirt_detection.pt")

    async def connect(self):
        """Connect to the camera WebSocket and start listening for messages."""
        print("Connecting to camera websocket with URL: ", CAMERA_WEBSOCKET_URL)
        self.websocket = await websockets.connect(CAMERA_WEBSOCKET_URL)
        asyncio.create_task(self._broadcast_forever())

    
    async def _broadcast_forever(self):
        """Listen for messages from the WebSocket of the camera and broadcast them to all connected clients."""
        try:
            async for message in self.websocket: # type: ignore
                # print(f"Received message from robot: {message}")
                await self.process_message(message)
        except ConnectionClosedError:
            print("Connection to camera has been lost, attempting to reconnect...")
            await self._broadcast(json.dumps({ "type": "SPacketError", "data": {
                "message": "Conexão com a câmera foi perdida"
            }}))
            await self.reconnect()

    async def process_message(self, message):
        """Process the message received from the camera, run YOLO model and broadcast results."""
        data = json.loads(message)
        if "bytes" in data:
            img_data = base64.b64decode(data["bytes"])
            np_arr = np.frombuffer(img_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            results = self.yolo_model.predict(img)  # Rodar o modelo YOLO

            # print("Results:", results)

            results = await asyncio.to_thread(self.yolo_model.predict, img)
            results = results[0]

            # print("Results:", results)

            # Printar o probs
            print("Probs:", results.probs)
            
            # Detectar se foi detectado sujeira ou não
            if results.names[0] is not None:
                await self._broadcast(json.dumps({ "type": "SPacketInfo", "data": {
                    "message": "Sujeira detectada"
                }}))

    async def add_client(self, client: WebSocket):
        self.clients.add(client)
        print(f"Added client {client}")
    
    async def remove_client(self, client: WebSocket):
        self.clients.remove(client)
        print(f"Removed client {client}")
    
    async def send(self, data):
        """Send a message to the camera."""
        if self.websocket is None:
            raise Exception("Not connected to camera")

        await self.websocket.send(data)
    
    async def _broadcast(self, message):
        # print(f"Broadcasting message to {len(self.clients)} clients: {message}")
        if self.clients:
            await asyncio.gather(*[client.send_text(message) for client in self.clients])
    
    async def _reconnect(self):
        if self.websocket:
            try: await self.close()
            except: pass

        try: await self.connect()
        except Exception as e:
            print(f"Failed to reconnect to camera: {e}, retrying in 5 seconds...")
            await asyncio.sleep(5)
            return await self.reconnect()

        await self._broadcast(json.dumps({ "type": "SPacketInfo", "data": {
            "message": "Conexão com a câmera estabelecida"
        }}))
        print("Connected to camera")

    async def reconnect(self):
        """Close the current connection and reconnect to the camera WebSocket."""
        # use _reconnect in a non-blocking way
        asyncio.create_task(self._reconnect())

    async def close(self):
        if self.websocket:
            await self.websocket.close()
        self.websocket = None

camera = Camera()
```

Nesse client, que pode ser encontrado em (\2024-1B-T08-EC06-G01\src\backend\src\client\camera.py), é feita a conexão com a câmera do robô, onde é feita a detecção de sujeira no ambiente. Como foi explicado com mais detalhes na sprint passada, nos temops um websocket no robô que envia os dados da câmera. Agora nos estamos também recebendo esses dados no backend, onde são processadas pelo modelo YOLO, que nos treinamos, para detectar a presença de sujeira. Os resultados da detecção são enviados para o frontend para que o usuário possa visualizar o estado do ambiente e tomar decisões com base nesses dados. 

Para tentar ao maximo diminuir a latência, não estamos criando as caixas por volta dos objetos detectados, apenas estamos enviando se foi detectado sujeira ou não, onde o frontend vai exibir apenas uma mensagem de alerta. Na proxima sprint, vamos tentar melhorar a detecção, otimizar mais o modelo e exportar o modelo para outro formato para que não tenhamos que usar a biblioteca Ultralytics, já que ela é muito pesada.
