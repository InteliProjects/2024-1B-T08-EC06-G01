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

