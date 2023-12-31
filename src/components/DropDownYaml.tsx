"use client";

import { useRef } from "react";
import { FileText, Folder } from "lucide-react";

import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuGroup,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuPortal,
  DropdownMenuSeparator,
  DropdownMenuSub,
  DropdownMenuSubContent,
  DropdownMenuSubTrigger,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import type { FolderStructure } from "@/components/tabComponents/YamlEdit";

import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "./ui/tooltip";

export function DropDownYaml({
  folderStructure,
  handleParseConfig,
}: {
  handleParseConfig: (path: string) => void;
  folderStructure: FolderStructure;
}) {
  const handleSelect = (path: string) => {
    handleParseConfig(path);
  };

  const renderFolderStructure = (
    data: FolderStructure,
    currentPath = "config"
  ) => {
    const items = [];

    for (const key in data) {
      if (Array.isArray(data[key])) {
        for (const file of data[key] as string[]) {
          items.push(
            <DropdownMenuItem
              key={key + "/" + file}
              onSelect={() => {
                const newPath = currentPath + "/" + file;
                handleSelect(newPath);
              }}
            >
              <FileText className="mr-2 h-4 w-4" />
              <span>{file}</span>
            </DropdownMenuItem>
          );
        }
      } else {
        items.push(
          <DropdownMenuSub key={key}>
            <DropdownMenuSubTrigger>
              <Folder className="mr-2 h-4 w-4" />
              <span>{key}</span>
            </DropdownMenuSubTrigger>
            <DropdownMenuPortal>
              <DropdownMenuSubContent>
                {renderFolderStructure(
                  data[key] as FolderStructure,
                  currentPath + "/" + key
                )}
              </DropdownMenuSubContent>
            </DropdownMenuPortal>
          </DropdownMenuSub>
        );
      }
    }

    return items;
  };

  const triggerRef = useRef<HTMLButtonElement>(null);

  return (
    <DropdownMenu>
      <TooltipProvider>
        <Tooltip>
          <TooltipTrigger>
            <DropdownMenuTrigger asChild ref={triggerRef}>
              <Button
                onClick={() => {
                  triggerRef.current?.click();
                }}
                className="w-fit p-2"
                variant="outline"
              >
                Select Config File
              </Button>
            </DropdownMenuTrigger>
          </TooltipTrigger>
          <TooltipContent>
            <span className="font-mono text-sm">
              Select YAML config files from
              /src/launcher/autoware_launch/autoware_launch/config to edit
            </span>
          </TooltipContent>
        </Tooltip>
      </TooltipProvider>
      <DropdownMenuContent className="ml-8 p-4">
        <DropdownMenuLabel>Select a YAML file</DropdownMenuLabel>
        <DropdownMenuSeparator />
        <DropdownMenuGroup>
          {renderFolderStructure(folderStructure)}
        </DropdownMenuGroup>
      </DropdownMenuContent>
    </DropdownMenu>
  );
}
