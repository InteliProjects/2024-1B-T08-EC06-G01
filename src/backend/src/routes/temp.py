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

def generate_heatmap(data):
    if not data:
        return None
    x, y, temp = zip(*data)
    heatmap, xedges, yedges = np.histogram2d(x, y, bins=(50, 50), weights=temp, density=True)
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]

    plt.clf()
    plt.imshow(heatmap.T, extent=extent, origin='lower', cmap='hot')
    plt.colorbar()

    buf = BytesIO()
    plt.savefig(buf, format='png')
    buf.seek(0)
    img_base64 = base64.b64encode(buf.read()).decode('utf-8')
    return img_base64
