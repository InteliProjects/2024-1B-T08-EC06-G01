<template>
  <div class="container">
    <div class="column">
      <h2 class="title">Histórico de Dados</h2>
      <p class="subtitle">Último heatmap gerado:</p>
      <div class="line"></div>
      <div class="heatmap" v-if="heatmapImage">
        <img :src="'data:image/png;base64,' + heatmapImage" alt="Heatmap" />
      </div>
      <div v-else>Carregando...</div>
    </div>
    <div class="column2">
      <div class="dropdown">
        <button class="dropdown-toggle" @click="toggleDropdown">
          {{ selectedOption || 'Ordenar por' }}
          <i class="fas fa-chevron-down" :class="{ 'rotated': isOpen }"></i>
        </button>
        <div v-if="isOpen" class="dropdown-menu">
          <a href="#" class="dropdown-item" @click.prevent="selectOption('Data Crescente')">Data Crescente</a>
          <a href="#" class="dropdown-item" @click.prevent="selectOption('Data Decrescente')">Data Decrescente</a>
        </div>
      </div>
      <table class="table">
        <thead>
          <tr>
            <th>Data</th>
            <th>Localização no eixo X</th>
            <th>Localização no eixo Y</th>
            <th>Temperatura</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="item in items" :key="item.id">
            <td>{{ item.date }}</td>
            <td>{{ item.location_x }}</td>
            <td>{{ item.location_y }}</td>
            <td>{{ item.temp }}</td>
          </tr>
        </tbody>
      </table>
      <nav class="pagination">
        <ul class="flex items-center -space-x-px h-10 text-base">
          <li>
            <a href="#" @click.prevent="changePage(currentPage - 1)" class="flex items-center justify-center px-4 h-10 leading-tight text-black bg-green-400 border border-green-300 hover:bg-green-100 hover:text-gray-700" :class="{ 'disabled': currentPage === 1 }">
              <span class="sr-only">Previous</span>
              <svg class="w-3 h-3 rtl:rotate-180" aria-hidden="true" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 6 10">
                <path stroke="currentColor" stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M5 1 1 5l4 4"/>
              </svg>
            </a>
          </li>
          <li v-for="page in totalPages" :key="page">
            <a href="#" @click.prevent="changePage(page)" class="flex items-center justify-center px-4 h-10 leading-tight text-black bg-green-300 border border-green-200 hover:bg-green-100 hover:text-gray-700" :class="{ 'active': currentPage === page }">{{ page }}</a>
          </li>
          <li>
            <a href="#" @click.prevent="changePage(currentPage + 1)" class="flex items-center justify-center px-4 h-10 leading-tight text-black bg-green-400 border border-green-300 hover:bg-green-100 hover:text-gray-700" :class="{ 'disabled': currentPage === totalPages }">
              <span class="sr-only">Next</span>
              <svg class="w-3 h-3 rtl:rotate-180" aria-hidden="true" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 6 10">
                <path stroke="currentColor" stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="m1 9 4-4-4-4"/>
              </svg>
            </a>
          </li>
        </ul>
      </nav>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted } from 'vue';
import axios from 'axios';

const isOpen = ref(false);
const selectedOption = ref(null);
const heatmapImage = ref(null);
const currentPage = ref(1);
const itemsPerPage = ref(5);
const items = ref([]); // Defina items como uma variável reativa aaaaaa

function toggleDropdown() {
  isOpen.value = !isOpen.value;
}

async function fetchData() {
  try {
    const response = await axios.get('http://localhost:8000/temp/list');
    if (response.data.error) {
      console.error('Erro ao buscar dados:', response.data.message);
      return;
    }
    items.value = response.data.data; // Atualiza os dados recebidos da API
    heatmapImage.value = response.data.heatmap;
    console.log(response.data.data); // Exibe os dados recebidos no console (opcional)
  } catch (error) {
    console.error('Erro ao buscar dados:', error);
  }
}

function selectOption(option) {
  selectedOption.value = option;
  isOpen.value = false;
  sortItems();
}

function sortItems() {
  if (selectedOption.value === 'Data Crescente') {
    items.value.sort((a, b) => new Date(a.date) - new Date(b.date));
  } else if (selectedOption.value === 'Data Decrescente') {
    items.value.sort((a, b) => new Date(b.date) - new Date(a.date));
  }
}

// Computed property para calcular o número total de páginas
const totalPages = computed(() => {
  return Math.ceil(items.value.length / itemsPerPage.value);
});

function changePage(page) {
  if (page < 1 || page > totalPages.value) return;
  currentPage.value = page;
}

onMounted(() => {
  fetchData(); // Chama fetchData() quando o componente é montado
});
</script>


<style scoped>
.container {
  display: flex;
}

.column {
  padding-left: 12rem;
  padding-top: 1rem;
}

.column2 {
  padding-right: 20rem;
  padding-top: 1rem;
  width: 52rem;
}

.title {
  font-family: 'Poppins';
  text-align: left;
  font-size: 2.3rem;
}

.subtitle {
  font-family: 'Public sans';
  text-align: left;
  font-size: 1.25rem;
  padding-top: 1.8rem;
}

.line {
  margin-top: 0.2rem;
  height: 0.16rem;
  width: 13.8rem;
  background-color: #369C62;
}

.heatmap {
  margin-top: 4rem;
  width: 28rem;
  height: 28rem;
  border-radius: 50%;
  background-clip: padding-box;
}

.dropdown {
  position: relative;
  display: inline-block;
  padding-top: 6.5rem;
  padding-right: 0.6rem;
  padding-left: 8rem;
}

.dropdown-toggle {
  background-color: #4CDD8B;
  color: black;
  padding: 0.4rem;
  font-size: 0.85rem;
  font-family: Poppins;
  border: none;
  cursor: pointer;
  width: 10.56rem;
  text-align: left;
  padding-left: 0.9rem;
  border-radius: 0.2rem;
  position: relative;
  padding-right: 2rem;
}

.dropdown-toggle i {
  position: absolute;
  top: 50%;
  right: 0.6rem;
  transform: translateY(-50%);
  transition: transform 0.3s;
}

.rotated {
  transform: translateY(-50%) rotate(180deg);
}

.dropdown-menu {
  display: block;
  font-size: 0.85rem;
  font-family: Poppins;
  position: absolute;
  background-color: white;
  min-width: 10rem;
  box-shadow: 0rem 0.5rem 1rem 0rem rgba(0,0,0,0.2);
  z-index: 1;
}

.dropdown-item {
  color: black;
  padding: 0.75rem 1rem;
  text-decoration: none;
  display: block;
  text-align: left;
  width: 10.56rem;
}

.dropdown-item:nth-child(even) {
  background-color: #E4FFF0; 
}

.dropdown-item:nth-child(odd) {
  background-color: #D1FDE4; 
}

.dropdown-item:hover {
  background-color: #f1f1f1;
}

.dropdown:hover .dropdown-menu,
.dropdown-menu:hover {
  display: block;
}

.table {
  margin-left: 8rem;
  width: 44rem;
  border-collapse: collapse;
  margin-top: 4rem;
  cursor: pointer;
}

.table th, .table td {
  border: 0.06rem solid;
  padding: 0.5rem;
  padding-left: 1rem;
  padding-right: 1rem;
  text-align: center;
  font-family: Poppins;
  font-weight: normal;
}

.table th {
  background-color: #ffffff;
}

.table tbody tr:nth-child(even) {
  background-color: #ffffff;
}

.table tbody tr:hover {
  background-color: #E4FFF0;
}

.table tbody tr:nth-child(odd) {
  height: 1.8rem;
}

.table tbody tr:nth-child(even) {
  height: 1.8rem;
}

.pagination {
  margin-top: 2rem;
  margin-left: -5rem;
  display: flex;
  justify-content: center;
}

.pagination a.disabled {
  pointer-events: none;
  opacity: 0.6;
}

.pagination a.active {
  background-color: #4CDD8B;
  color: white;
}
</style>
