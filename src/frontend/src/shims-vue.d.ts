// Esse arquivo tem o objetivo de ajudar o TypeScript a entender arquivos .vue 

declare module '*.vue' {
    import type { DefineComponent } from 'vue';
    const component: DefineComponent<{}, {}, any>;
    export default component;
  }