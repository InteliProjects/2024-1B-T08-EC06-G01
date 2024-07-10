import ormar
import ormar.exceptions
from fastapi import APIRouter
from fastapi.responses import JSONResponse
from models.robots import Robot as RobotModel
from schemas.robots import Robot

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




