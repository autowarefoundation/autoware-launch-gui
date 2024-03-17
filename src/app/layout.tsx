"use client";

import "@/styles/globals.css";

import { useEffect } from "react";
import dynamic from "next/dynamic";
import { invoke } from "@tauri-apps/api/core";
import { Window } from "@tauri-apps/api/window";
import { Provider } from "jotai";

import { cn } from "@/lib/utils";
import { Toaster } from "@/components/ui/toaster";
import { StyleSwitcher } from "@/components/style-switcher";
import { ThemeProvider } from "@/components/theme-provider";
import WebSocketComponent from "@/components/WebSocket";

const Menu = dynamic(
  () => import("../components/menu").then((mod) => mod.Menu),
  { ssr: false }
);

const isWindow = typeof window !== undefined;

interface ExamplesLayoutProps {
  children: React.ReactNode;
}

// add ctrl+r to reload page
// add ctrl+shift+r to reload page and clear cache

export default function MyApp({ children }: ExamplesLayoutProps) {
  useEffect(() => {
    const handleKeyPress = async (e: KeyboardEvent) => {
      if (e.ctrlKey && e.key === "r") {
        e.preventDefault();
        window.location.reload();
      } else if (e.ctrlKey && e.key === "q") {
        e.preventDefault();
        // @ts-ignore
        if (!(isWindow && window.__TAURI__)) {
          return;
        }
        await invoke("kill_autoware_process", {});

        await Window.getCurrent().destroy();
      }
    };
    window.addEventListener("keypress", handleKeyPress);
    return () => {
      window.removeEventListener("keypress", handleKeyPress);
    };
  }, []);
  return (
    <html
      lang="en"
      suppressHydrationWarning
      className="overflow-clip rounded-xl bg-black"
    >
      <head />
      <body className="overflow-clip bg-transparent font-sans antialiased scrollbar-none">
        <Provider>
          <ThemeProvider attribute="class" defaultTheme="system" enableSystem>
            <div className={`max-h-screen overflow-clip rounded-lg border-2`}>
              <Menu />
              <Toaster />
              <WebSocketComponent />
              <div
                className={cn(
                  "h-screen overflow-auto border-t-2 bg-background pb-8",
                  // "scrollbar-none",
                  "scrollbar-thin scrollbar-track-transparent scrollbar-thumb-accent scrollbar-thumb-rounded-md"
                )}
              >
                {children}
              </div>
            </div>
            {/* <TailwindIndicator /> */}
          </ThemeProvider>
          <StyleSwitcher />
        </Provider>
      </body>
    </html>
  );
}

// export const metadata: Metadata = {
//   icons: {
//     shortcut: ["#"],
//   },
// }
