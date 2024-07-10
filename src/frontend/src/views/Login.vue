<template>
  <div class="container">
      <div class="square">
          <div class="column">
              <h2 class="title">Fazer log-in</h2>
              <p class="subtitle">Username:</p>
              <div class="input-box">
                  <input id="username" type="text" class="text-input" placeholder="Digite seu username" />
              </div>
              <p class="subtitle2">Senha:</p>
              <div class="input-box">
                  <input id="senha" type="password" class="text-input" placeholder="Digite sua senha de acesso" />
              </div>
              <div class="button-container">
                  <button class="cadastre-button" @click="goToCadastre">Cadastrar</button>
                  <button class="enter-button" @click="loginUser">Entrar</button>
              </div>
          </div>
          <div class="column2">
            <div class="square2">
              <div class="bananeira">
                <Bananeira />
              </div>
            </div>
          </div>
      </div>
  </div>
</template>

<script setup lang="ts">
import { useRouter } from 'vue-router';
import axios from 'axios';

const router = useRouter();

async function loginUser() {
  const username = (document.getElementById('username') as HTMLInputElement).value;
  const password = (document.getElementById('senha') as HTMLInputElement).value;

  console.log(username, password);

  const data = {
    username,
    password
  };

  await axios.post('http://localhost:8000/users/login', data)
    .then(res => {
      console.log(res.data);
      if (!("error" in res.data)){
        router.push(`/home?id=${res.data.id}`);
        return;
      }
      alert('Usuário ou senha inválidos');
    })
    .catch(error => {
      console.log(error);
    });
}

function goToCadastre() {
  router.push('/');
}

import Bananeira from '../assets/bananeira.svg';
</script>


<style scoped>
.container {
  display: flex;
  justify-content: center;
  align-items: center;
  height: 100vh;
}

.square{
    width: 80%;
    height: 40rem;
    background-color: #fff;
    box-shadow: 0 0 15px rgba(0, 0, 0, 0.3);
    border-radius: 1.25rem;
    position: relative;

}

.square2{
  position: absolute;
  top: 0;
  right: 0;
  width: 50%;
  height: 100%;
  background: linear-gradient(180deg, #EDFFF5, #D3FFE6);
  border-radius: 0 1.25rem 1.25rem  0;
}

.bananeira{
  padding-top: 10rem;
  padding-left: 9rem;
}

.column {
  padding-left: 6.3rem;
  padding-top: 5rem;
}

.column2{
  padding-left: 24rem;
}

.title {
  font-family: 'Poppins';
  text-align: left;
  font-size: 2.4rem;
  color: #0C8541;
}

.subtitle {
  font-family: 'Public sans';
  color: #0C8541;
  text-align: left;
  font-size: 1.25rem;
  padding-top: 3rem;
}

.subtitle2 {
  font-family: 'Public sans';
  color: #0C8541;
  text-align: left;
  font-size: 1.25rem;
  padding-top: 2.5rem;
}

.input-box {
  margin-top: 0.8rem;
}

.text-input {
  width: 36.5%;
  padding: 0.5rem;
  font-size: 1rem;
  border: 1px solid #DBFFEB;
  border-radius: 0.5rem;
  box-sizing: border-box;
  background-color: #EDFFF5;
}

.text-input:focus {
  border-color: #0C8541;
  outline: none;
  box-shadow: 0 0 5px rgba(12, 133, 65, 0.5);
}

.button-container {
  margin-top: 2rem;
}

.cadastre-button {
  color: #0C8541;
  padding: 2.4rem 3.6rem;
  margin-right: 1rem;
  font-size: 1.18rem;
  font-family: 'Poppins', sans-serif;
  border: none;
  border-radius: 0.25rem;
  cursor: pointer;
  text-decoration: underline;
}

.enter-button{
  color: #0C8541;
  background-color: #fff;
  padding: 0.4rem 1.8rem;
  font-size: 1.18rem;
  font-family: 'Poppins', sans-serif;
  border: 1px solid #0C8541;
  border-radius: 0.25rem;
  cursor: pointer;

}

.enter-button:hover {
  background-color: #0a6b37;
  color: #fff;
}

</style>