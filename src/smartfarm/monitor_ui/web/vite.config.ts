import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

export default defineConfig({
  plugins: [react()],
  // host: true 라야 같은 망의 다른 PC에서도 열린다(데모 때 노트북으로 확인).
  server: { host: true, port: 5173 },
});
