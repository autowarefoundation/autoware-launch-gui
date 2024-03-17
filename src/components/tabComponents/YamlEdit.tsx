"use client";

import React from "react";
import { invoke } from "@tauri-apps/api/core";
import { useAtom } from "jotai";

import {
  autowareFolderPathAtom,
  parsedConfigFolderStructureAtom,
} from "@/app/jotai/atoms";

import { YamlArgsDialog } from "../DialogYamlParser";
import { DropDownYaml } from "../DropDownYaml";

export type FolderStructure = { [key: string]: FolderStructure | string[] };

const isWindow = typeof window !== "undefined";

function generateFolderStructure(paths: string[]): FolderStructure {
  const root: FolderStructure = {};

  for (const path of paths) {
    // Split the path into its components
    const parts = path.split("/");

    // Find the index of 'config' to use it as the root
    const configIndex = parts.indexOf("config");
    if (configIndex === -1) continue; // If 'config' is not in the path, skip it

    // Filter out only the relevant parts after 'config' and only consider .yaml or .yml files
    const relevantParts = parts
      .slice(configIndex + 1)
      .filter(
        (part) =>
          part.endsWith(".yaml") || part.endsWith(".yml") || !part.includes(".")
      );

    let currentLevel = root;

    for (let i = 0; i < relevantParts.length; i++) {
      const part = relevantParts[i];

      // If it's a yaml/yml file, add it to the current level
      if (part.endsWith(".yaml") || part.endsWith(".yml")) {
        if (!currentLevel["files"]) {
          currentLevel["files"] = [];
        }
        (currentLevel["files"] as string[]).push(part);
      } else {
        // If it's a folder, go one level deeper
        if (!currentLevel[part]) {
          currentLevel[part] = {};
        }
        currentLevel = currentLevel[part] as FolderStructure;
      }
    }
  }

  return root;
}

const YAMLEdit = () => {
  const [autowareFolderPath, setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  );

  const [parsedConfig, setParsedConfig] = React.useState<{
    [key: string]: any;
  } | null>(null);
  const [parsedConfigPath, setParsedConfigPath] = React.useState<string>("");

  const [parsedConfigFolderStructure, setParsedConfigFolderStructure] = useAtom(
    parsedConfigFolderStructureAtom
  );

  const configFolderPath = `${autowareFolderPath}/src/launcher/autoware_launch/autoware_launch/config`;
  const findConfigFiles = async () => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    const res: string[] = await invoke("find_yaml_files", {
      path: configFolderPath,
    });

    setParsedConfigFolderStructure(generateFolderStructure(res));
  };

  const handleParseConfig = async (path: string) => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    const absolutePath = `${configFolderPath.replace(/\/config$/, "")}/${path}`;

    setParsedConfig(null);
    // setParsedConfigPath("");

    const text: any = await invoke("parse_yaml", {
      path: absolutePath,
    });
    setParsedConfig(text);
    setParsedConfigPath(absolutePath);
  };

  React.useEffect(() => {
    if (parsedConfigPath === "") {
      setParsedConfig(null);
    } else {
      const filePathAfterConfig = parsedConfigPath.replace(
        configFolderPath,
        "config"
      );
      handleParseConfig(filePathAfterConfig);
    }
  }, [parsedConfigPath]);

  React.useEffect(() => {
    if (autowareFolderPath) {
      findConfigFiles();
    }
  }, [autowareFolderPath]);

  return (
    <div className="flex flex-col items-start gap-4">
      <div className="flex w-full items-center gap-4 text-xl font-medium">
        {parsedConfigFolderStructure ? (
          <DropDownYaml
            handleParseConfig={handleParseConfig}
            folderStructure={parsedConfigFolderStructure}
          />
        ) : null}
        {parsedConfigPath ? (
          <div className="flex items-center gap-2 self-start text-xl font-medium">
            <div>Parsed Config File:</div>
            <span className="h-10 w-fit overflow-x-auto rounded-md border border-input p-2 font-mono text-sm font-normal">
              {parsedConfigPath.split("/").pop()}
            </span>
          </div>
        ) : null}
        {parsedConfig ? (
          <YamlArgsDialog
            setConfigPath={setParsedConfigPath}
            yamlData={parsedConfig}
            path={parsedConfigPath}
          />
        ) : null}
      </div>
    </div>
  );
};

export default YAMLEdit;
