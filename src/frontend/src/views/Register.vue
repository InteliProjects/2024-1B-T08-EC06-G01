<template>
  <div class="container">
    <div class="column">
      <h2 class="title">Registros de Uso</h2>

      <!-- A lógica do filtro não está implementada -->
      <div class="checkbox-container">
        <input type="checkbox" id="checkbox" v-model="isChecked">
        <label for="checkbox">Agrupar por usuário</label>
      </div>
      <table class="table">
        <thead>
          <tr>
            <th>Usuário</th>
            <th>Data</th>
            <th>Horário</th>
            <th>Uso do Botão de Emergência</th>
            <th>Requisição da IA</th>
          </tr>
        </thead>
        <tbody>
          <template v-if="!isChecked">
            <!-- Exibição padrão sem agrupamento -->
            <tr v-for="log in logs" :key="log.id">
              <td>{{ log.username }}</td>
              <td>{{ formatDate(log.date) }}</td>
              <td>{{ formatTime(log.date) }}</td>
              <td>{{ log.emergency_stop ? 'Sim' : 'Não' }}</td>
              <td>{{ log.ia_request ? 'Sim' : 'Não' }}</td>
            </tr>
          </template>
          <template v-else>
          <!-- Exibição quando agrupado por usuário -->
          <tr v-for="log in groupedLogs" :key="log.id">
            <td>{{ log.username }}</td>
            <td>{{ formatDate(log.date) }}</td>
            <td>{{ formatTime(log.date) }}</td>
            <td>{{ log.emergency_stop ? 'Sim' : 'Não' }}</td>
            <td>{{ log.ia_request ? 'Sim' : 'Não' }}</td>
          </tr>
        </template>
        </tbody>
      </table>
    </div>
  </div>
</template>


<script>
import { ref, onMounted, computed } from 'vue';
import axios from 'axios';
import { format } from 'date-fns';

export default {
  name: 'Register',
  setup() {
    const isChecked = ref(false);
    const logs = ref([]);
    const groupedLogs = ref([]);

    // Função para formatar a data
    const formatDate = (dateString) => {
      return format(new Date(dateString), 'dd/MM/yyyy');
    };

    // Função para formatar a hora
    const formatTime = (dateString) => {
      return format(new Date(dateString), 'HH:mm:ss');
    };

    // Chama a API quando o componente é montado
    onMounted(async () => {
      try {
        const res = await axios.get('http://localhost:8000/logs/list');
        logs.value = res.data.data;

        const local_logs = logs.value.map(log => [log])

        const users = logs.value.map(log => Number(log.user_id.id)).sort()
        const uniqueUsers = [...new Set(users)]

        let grouped = []

        uniqueUsers.forEach(user => {
          const userLogs = logs.value.filter(log => log.user_id.id === user)
          userLogs.forEach(log => {
            grouped.push(log)
          })
        })

        groupedLogs.value = grouped
      } catch (error) {
        console.log(error);
      }
    });

    return {
      isChecked,
      logs,
      groupedLogs,
      formatDate,
      formatTime
    };
  }
};
</script>



<style scoped>
  .container {
    display: flex;
  }

  .column {
    padding-left: 12rem;
    padding-top: 1rem;
  }

  .title {
    font-family: 'Poppins';
    text-align: left;
    font-size: 2.3rem;
  }

  .checkbox-container {
    display: flex;
    align-items: center;
    margin-bottom: 1rem;
    margin-top: 1.8rem;
  }

  .checkbox-container input[type="checkbox"] {
    margin-right: 1.6rem;
    width: 1.1rem;
    height: 1.1rem;
    -webkit-appearance: none;
    -moz-appearance: none;
    appearance: none;
    border: 1px solid black;
    position: relative;
  }

  .checkbox-container input[type="checkbox"]::before {
    content: "";
    position: absolute;
    width: 1rem;
    height: 1rem;
    background: #17CE67;
    display: none;
    justify-content: center;
    align-items: center;
    font-size: 0.8rem;
  }

  .checkbox-container input[type="checkbox"]:checked::before {
    display: flex;
    content: "\2713";
    color: white;
  }

  .table {
    width: 80rem;
    border-collapse: collapse;
    margin-top: 4rem;
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
</style>
