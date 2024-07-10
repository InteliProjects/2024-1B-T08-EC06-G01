<template>
  <div class="main flex items-center justify-center min-h-screen">
    <div class="flex xl:mx-[9rem] mx-0 xl:my-[rem] my-0 container rounded-2xl shadow-2xl">
      <div class="hidden xl:flex xl:w-1/2 bg-[#ffedd7] xl:py-[1rem] py-0 xl:px-[8em] px-0 justify-center items-center rounded-tl-2xl rounded-bl-2xl overflow-hidden">
        <Bananeira />
      </div>
      <div class="w-full xl:w-1/2 rounded-tr-2xl rounded-br-2xl bg-white xl:px-0 px-0 xl:py-2 py-0">
        <div class="xl:mr-24 mr-5 xl:ml-20 ml-5">
          <h1 class="text-[2.4rem] text-[#F28705] pb-8 mt-4 text-center pt-8 xl:pt-0 xl:text-start">Criar Conta</h1>
          <div class="pb-4">
            <h2 class="text-[1.25rem] text-[#F28705] pb-2">Nome:</h2>
            <input type="text" id="nome_completo" class="bg-[#FFF4E7]  text-gray-900 text-sm rounded-xl block w-full py-3 px-2.5 focus:border-[#F28705] focus:outline-none" placeholder="Digite seu nome completo" required />
          </div>
          <div class="pb-5">
            <h2 class="text-[1.25rem] text-[#F28705] pb-2">Username:</h2>
            <input type="text" id="username" class="bg-[#FFF4E7] text-gray-900 text-sm rounded-xl block w-full py-3 px-2.5 focus:border-[#F28705] focus:outline-none" placeholder="Crie seu username" required />
          </div>
          <div class="pb-5">
            <h2 class="text-[1.25rem] text-[#F28705] pb-2">Senha:</h2>
            <input type="password" id="senha" class="bg-[#FFF4E7]  text-gray-900 text-sm rounded-xl block w-full py-3 px-2.5 focus:border-[#F28705] focus:outline-none" placeholder="Digite uma senha de acesso" required />
          </div>
          <div class="flex flex-col justify-center items-center">
            <a href="/login" class="pt-10 pb-7">
              <h1 class="text-[1.25rem] underline text-[#F28705]">Já tenho cadastro</h1>
            </a>
            <button @click="register_user" class="hover:bg-[#F28705] text-[1.18rem] hover:text-white text-[#F28705] font py-2 px-6 rounded-lg border border-[#F28705] mb-10">
              Cadastrar
            </button>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import Bananeira from '../assets/bananeira.svg'
import axios from 'axios';
</script>

<script>
export default {
  name: 'Signup',
  methods: {
    async register_user() {
          const nome_completo = document.getElementById('nome_completo').value;
          const username = document.getElementById('username').value;
          const password = document.getElementById('senha').value;
    
          const data = {
            username,
            password
          }
    
          await axios.post('http://localhost:8000/users/register', data)
           .then(res => {
              console.log(res.data);
              alert('Usuário cadastrado com sucesso');
            })
           .catch(error => {
              alert('Erro ao cadastrar usuário, verifique se esse usuario ja nao existe');
              console.log(error);
            })
        }
  }
}
</script>

<style scoped>
.main {
  font-family: 'Poppins';
}
.container {
  box-shadow: 0 0 15px rgba(0, 0, 0, 0.3);
}
</style>
