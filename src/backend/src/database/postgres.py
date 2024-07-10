import os

import databases
from dotenv import load_dotenv
from ormar import OrmarConfig
from sqlalchemy.sql.schema import MetaData

load_dotenv("./.env")

DATABASE_URL = os.environ.get("DATABASE_URL")
assert DATABASE_URL is not None, "DATABASE_URL is not set"

metadata = MetaData()
database = databases.Database(DATABASE_URL)

base_ormar_config = OrmarConfig(
	database=database,
	metadata=metadata
)