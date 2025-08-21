cd interceptor/
python -m venv .venv

Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
MultiSense-UAV-Detection\interceptor> .venv\Scripts\activate

pip freeze > requirements.txt