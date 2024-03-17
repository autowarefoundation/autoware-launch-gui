"use client";

import React, { useEffect, useRef, useState } from "react";
import { open } from "@tauri-apps/plugin-dialog";
import { readTextFile, writeTextFile } from "@tauri-apps/plugin-fs";

import { Button } from "./ui/button";
import { Textarea } from "./ui/textarea";
import { toast } from "./ui/use-toast";

const isWindow = typeof window !== undefined;

const ReadConfig: React.FC = () => {
  const [fileContent, setFileContent] = useState<string | null>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);
  const [filePath, setFilePath] = useState<string | null>(null);

  const handleFileRead = async () => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    try {
      // Use the dialog plugin to open a file picker
      const paths = await open({
        multiple: false,
        directory: false,
        filters: [{ name: "YAML", extensions: ["yaml", "yml"] }],
      });
      if (paths) {
        // Use the fs plugin to read the selected file
        const content = await readTextFile(paths.path);
        setFileContent(content);
        setFilePath(paths.path);
      }
    } catch (error) {
      console.error("Error reading the file:", error);
    }
  };

  const handleFileSave = async () => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    if (filePath && fileContent) {
      try {
        await writeTextFile(filePath, fileContent);
        toast({
          type: "foreground",
          description: "File saved!",
          variant: "default",
        });
      } catch (error) {
        console.error("Error writing to the file:", error);
      }
    } else {
      toast({
        type: "foreground",
        description: "No file selected!",
        variant: "destructive",
      });
    }
  };
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = "auto";
      textareaRef.current.style.height = `${textareaRef.current.scrollHeight}px`;
    }
  }, [fileContent]);

  return (
    <div className="flex w-full flex-col gap-4 p-4">
      <Button onClick={handleFileRead} className="w-fit">
        Read YAML File
      </Button>
      <Textarea
        ref={textareaRef}
        className="w-1/2 resize-none"
        value={fileContent || ""}
        onChange={(e) => {
          setFileContent(e.target.value);
          if (textareaRef.current) {
            textareaRef.current.style.height = "auto";
            textareaRef.current.style.height = `${textareaRef.current.scrollHeight}px`;
          }
        }}
      />
      <Button onClick={handleFileSave} className="mt-4 w-fit">
        Save Changes
      </Button>
    </div>
  );
};

export default ReadConfig;
