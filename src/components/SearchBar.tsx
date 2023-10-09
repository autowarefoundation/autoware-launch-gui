"use client"

import React from "react"

import { Input } from "@/components/ui/input"

export function Search({
  search,
  setSearch,
}: {
  search: string
  setSearch: React.Dispatch<React.SetStateAction<string>>
}) {
  return (
    <div className="flex items-center gap-4">
      <Input
        type="search"
        placeholder="Search..."
        className="w-[16rem]"
        value={search}
        onChange={(e) => setSearch(e.target.value)}
      />
    </div>
  )
}
