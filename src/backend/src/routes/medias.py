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
                    print(value)
                    media_dict[key] = str(value)
                elif isinstance(value, datetime):
                    media_dict[key] = value.isoformat()
            media_dicts.append(media_dict)
            print(media_dict)
        
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

        if not media:
            return JSONResponse(content={
                "error": True,
                "message": "Mídia não encontrada"
            }, status_code=404)
        
        media_dict = media.dict()
        for key, value in media_dict.items():
            if isinstance(value, uuid.UUID):
                media_dict[key] = str(value)
            if isinstance(value, datetime):
                media_dict[key] = value.isoformat()

        return JSONResponse(content={
            "error": False,
            "message": "Mídia encontrada",
            "data": media_dict
        }, status_code=200)
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

