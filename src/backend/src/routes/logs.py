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

# @router.post("/register")
# async def register(log: Log):
#     try:
#         await LogModel.objects.create(
#             date = log.date,
#             emergency_button = log.emergency_button,
#             ia_request = log.ia_request,
#             user_id = log.user_id
#         )
#         return JSONResponse(content={
#             "error": False,
#             "message": "Log criado com sucesso"
#         }, status_code=201)
#     except Exception as e:
#         return JSONResponse(content={
#             "error": True,
#             "message": f"Erro interno do servidor: {e}"
#         }, status_code=500)

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