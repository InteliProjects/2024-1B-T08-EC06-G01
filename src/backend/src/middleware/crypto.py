import os
from datetime import datetime
from typing import Literal

from dotenv import load_dotenv
from fastapi import Cookie, HTTPException
from jose import jwt

load_dotenv('./.env')

JWT_SECRET: str = os.environ.get('JWT_SECRET') or "secret"

def guard(token: str = Cookie(None)) -> Literal[True]:
    credentials_exception = HTTPException(
        status_code=401,
        detail="Não autorizado",
    )

    if token is None:
        raise credentials_exception

    try:
        payload = jwt.decode(token, secret, algorithms=["HS256"])
        username: str = payload.get("username")
        expiration: datetime = datetime.fromtimestamp(payload.get("exp"))

        if expiration is None:
            raise credentials_exception
        if datetime.utcnow() > expiration:
            raise credentials_exception

        if username is None:
            raise credentials_exception

    except jwt.PyJWTError:
        raise credentials_exception

    return True

