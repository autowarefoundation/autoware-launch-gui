"use client";

import React, { forwardRef } from "react";
import * as AccordionPrimitive from "@radix-ui/react-accordion";
import { readTextFile } from "@tauri-apps/plugin-fs";
import { useAtom } from "jotai";
import { ChevronRight, Cog, Folder, type LucideIcon } from "lucide-react";
import useResizeObserver from "use-resize-observer";

import { cn } from "@/lib/utils";
import { parsedLaunchFilePathAtom } from "@/app/jotai/atoms";

import { Button } from "./ui/button";
import { Input } from "./ui/input";

export interface ElementData {
  name: string;
  attributes: [string, string][];
  children?: ElementData[];
}
type TreeProps = React.HTMLAttributes<HTMLDivElement> & {
  data: ElementData[] | ElementData;
  initialSlelectedItemId?: string;
  onSelectChange?: (item: ElementData | undefined) => void;
  expandAll?: boolean;
  folderIcon?: LucideIcon;
  itemIcon?: LucideIcon;
};

const Tree = forwardRef<HTMLDivElement, TreeProps>(
  (
    {
      data,
      initialSlelectedItemId,
      onSelectChange,
      expandAll,
      folderIcon,
      itemIcon,
      className,
      ...props
    },
    ref
  ) => {
    const [selectedItemId, setSelectedItemId] = React.useState<
      string | undefined
    >(initialSlelectedItemId);

    const handleSelectChange = React.useCallback(
      (item: ElementData | undefined, index: number) => {
        setSelectedItemId(item ? item?.name + index : undefined);
        if (onSelectChange) {
          onSelectChange(item);
        }
      },
      [onSelectChange]
    );

    const expandedItemIds = React.useMemo(() => {
      if (!initialSlelectedItemId) {
        return [] as string[];
      }

      const ids: string[] = [];

      function walkTreeItems(
        items: ElementData[] | ElementData,
        targetId: string
      ) {
        if (items instanceof Array) {
          // eslint-disable-next-line @typescript-eslint/prefer-for-of
          for (let i = 0; i < items.length; i++) {
            ids.push(items[i]!.name);
            if (walkTreeItems(items[i]!, targetId) && !expandAll) {
              return true;
            }
            if (!expandAll) ids.pop();
          }
        } else if (!expandAll && items.name === targetId) {
          return true;
        } else if (items.children) {
          return walkTreeItems(items.children, targetId);
        }
      }

      walkTreeItems(data, initialSlelectedItemId);
      return ids;
    }, [data, initialSlelectedItemId]);

    const { ref: refRoot, width, height } = useResizeObserver();

    return (
      <div ref={refRoot} className={"overflow-hidden"}>
        <div className="relative p-2">
          <TreeItem
            data={data}
            ref={ref}
            selectedItemId={selectedItemId}
            handleSelectChange={handleSelectChange}
            expandedItemIds={expandedItemIds}
            FolderIcon={folderIcon}
            ItemIcon={itemIcon}
            {...props}
          />
        </div>
      </div>
    );
  }
);

type TreeItemProps = TreeProps & {
  selectedItemId?: string;
  handleSelectChange: (item: ElementData | undefined, index: number) => void;
  expandedItemIds: string[];
  FolderIcon?: LucideIcon;
  ItemIcon?: LucideIcon;
};
const isWindow = typeof window !== undefined;
const TreeItem = React.forwardRef<HTMLDivElement, TreeItemProps>(
  (
    {
      className,
      data,
      selectedItemId,
      handleSelectChange,
      expandedItemIds,
      FolderIcon,
      ItemIcon,
      ...props
    },
    ref
  ) => {
    const [parsedLaunchFilePath, _setParsedLaunchFilePath] = useAtom(
      parsedLaunchFilePathAtom
    );

    return (
      <div role="tree" className="rounded-md p-4 shadow-md" {...props}>
        <div className="list-decimal pl-5">
          {Array.isArray(data) ? (
            data.map((item, idx) => (
              <div
                key={item.name + idx}
                className={cn(
                  "my-2 flex cursor-pointer flex-col rounded-lg p-2 transition-all duration-300",
                  // only put the border on the children of this parent element
                  "border border-transparent hover:border-accent-foreground/50",
                  {
                    "border-accent-foreground/50": selectedItemId === item.name,
                    "border-transparent": selectedItemId !== item.name,
                  }
                )}
              >
                {item.children ? (
                  <AccordionPrimitive.Root
                    type="multiple"
                    defaultValue={expandedItemIds}
                    key={item.name + idx}
                  >
                    <AccordionPrimitive.Item value={item.name}>
                      <AccordionTrigger
                        className="flex w-full items-center rounded-md p-2 transition-all"
                        onClick={() => handleSelectChange(item, idx)}
                      >
                        {item.attributes.find((attr) => attr[0] === "name") && (
                          <Cog className="mr-2 h-4 w-4" aria-hidden="true" />
                        )}
                        {item.children.length > 0 && (
                          <Folder className="mr-2 h-4 w-4" aria-hidden="true" />
                        )}
                        <h2 className="text-lg font-semibold">{item.name}</h2>
                      </AccordionTrigger>
                      <AccordionContent className="mt-2 flex flex-col gap-2 pl-5">
                        <div className="flex flex-col gap-2 rounded-lg">
                          {item.attributes.map((attr, index) => (
                            <div
                              key={index}
                              className="mt-2 flex w-full items-center gap-4"
                            >
                              <span className="font-medium">{attr[0]}:</span>
                              <Input
                                type="text"
                                className="rounded-md border p-1"
                                defaultValue={attr[1]}
                              />
                            </div>
                          ))}
                          <Button
                            // only show this button if there is an input element in the same parent
                            className={`${
                              item.children.length > 0 ? "hidden" : ""
                            } w-fit self-end`}
                            // save changes
                            onClick={async () => {
                              // @ts-ignore
                              if (!(isWindow && window.__TAURI__)) {
                                return;
                              }
                              const textRead = await readTextFile(
                                parsedLaunchFilePath
                              );
                              const lines = textRead.split("\n");
                              const lineIndex = lines.findIndex((line) =>
                                line.includes(item.name)
                              );
                              const line = lines[lineIndex];
                            }}
                          >
                            Save Changes
                          </Button>
                          <TreeItem
                            data={item.children}
                            selectedItemId={selectedItemId}
                            handleSelectChange={handleSelectChange}
                            expandedItemIds={expandedItemIds}
                            FolderIcon={FolderIcon}
                            ItemIcon={ItemIcon}
                          />
                        </div>
                      </AccordionContent>
                    </AccordionPrimitive.Item>
                  </AccordionPrimitive.Root>
                ) : (
                  <Leaf
                    item={item}
                    isSelected={selectedItemId === item.name}
                    Icon={ItemIcon}
                  />
                )}
              </div>
            ))
          ) : (
            <div>
              <Leaf
                item={data}
                isSelected={selectedItemId === data.name}
                Icon={ItemIcon}
              />
            </div>
          )}
        </div>
      </div>
    );
  }
);

const Leaf = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement> & {
    item: ElementData;
    isSelected?: boolean;
    Icon?: LucideIcon;
  }
>(({ className, item, isSelected, Icon, ...props }, ref) => {
  return (
    <div
      ref={ref}
      className={
        "flex cursor-pointer items-center px-2 py-2         before:absolute before:left-0 before:right-1 before:-z-10 before:h-[1.75rem] before:w-full before:bg-muted/80 before:opacity-0 hover:before:opacity-100"
      }
      {...props}
    >
      {Icon && (
        <Icon
          className="mr-2 h-4 w-4 shrink-0 text-accent-foreground/50"
          aria-hidden="true"
        />
      )}
      <span className="flex-grow truncate text-sm">{item.name}</span>
    </div>
  );
});

const AccordionTrigger = React.forwardRef<
  React.ElementRef<typeof AccordionPrimitive.Trigger>,
  React.ComponentPropsWithoutRef<typeof AccordionPrimitive.Trigger>
>(({ className, children, ...props }, ref) => (
  <AccordionPrimitive.Header>
    <AccordionPrimitive.Trigger
      ref={ref}
      className={
        "flex w-full flex-1 items-center py-2 transition-all last:[&[data-state=open]>svg]:rotate-90"
      }
      {...props}
    >
      {children}
      <ChevronRight className="ml-4 h-4 w-4 shrink-0 text-accent-foreground/50 transition-transform duration-200" />
    </AccordionPrimitive.Trigger>
  </AccordionPrimitive.Header>
));
AccordionTrigger.displayName = AccordionPrimitive.Trigger.displayName;

const AccordionContent = React.forwardRef<
  React.ElementRef<typeof AccordionPrimitive.Content>,
  React.ComponentPropsWithoutRef<typeof AccordionPrimitive.Content>
>(({ className, children, ...props }, ref) => (
  <AccordionPrimitive.Content
    ref={ref}
    className={
      "overflow-hidden text-sm transition-all data-[state=closed]:animate-accordion-up data-[state=open]:animate-accordion-down"
    }
    {...props}
  >
    <div className="pb-1 pt-0">{children}</div>
  </AccordionPrimitive.Content>
));
AccordionContent.displayName = AccordionPrimitive.Content.displayName;

export { Tree };
