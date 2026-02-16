# Frontend Patterns

Advanced ChatKit JS frontend patterns: widgets, actions, theming, layout, and integration.

## Table of Contents

1. [Installation and Setup](#installation-and-setup)
2. [Theming Deep-Dive](#theming-deep-dive)
3. [Layout Patterns](#layout-patterns)
4. [Widgets](#widgets)
5. [Actions](#actions)
6. [File Uploads](#file-uploads)
7. [Thread Browsing](#thread-browsing)
8. [Next.js Integration](#nextjs-integration)
9. [Docusaurus Integration](#docusaurus-integration)

## Installation and Setup

### React

```bash
npm install @openai/chatkit-react
```

```tsx
import { ChatKit, useChatKit } from "@openai/chatkit-react";

function Chat() {
  const { control } = useChatKit({
    api: {
      url: "/api/chatkit",  // Your backend endpoint
      domainKey: "my-app",
    },
  });
  return <ChatKit control={control} />;
}
```

### CDN (Vanilla JS)

```html
<script src="https://cdn.platform.openai.com/deployments/chatkit/chatkit.js"></script>
```

### Self-hosted web component

```bash
npm install @openai/chatkit
```

```js
import "@openai/chatkit";
const el = document.createElement("openai-chatkit");
el.setOptions({ api: { url: "/api/chatkit", domainKey: "my-app" } });
document.getElementById("root").append(el);
```

## Theming Deep-Dive

### Basic theming

```tsx
<ChatKit
  control={control}
  theme={{
    colorScheme: "auto",       // "light" | "dark" | "auto"
    accentColor: "#6366f1",    // Indigo
  }}
/>
```

### Custom surface colors

```tsx
<ChatKit
  control={control}
  theme={{
    colorScheme: "dark",
    accentColor: "#10b981",
    surfaceColors: {
      background: "#0f172a",
      foreground: "#e2e8f0",
      border: "#334155",
      muted: "#1e293b",
    },
  }}
/>
```

### CSS class overrides

```tsx
<ChatKit control={control} className="my-chat-widget" />
```

```css
.my-chat-widget {
  --chatkit-radius: 12px;
  --chatkit-font-family: "Inter", sans-serif;
  border-radius: 16px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.12);
}
```

## Layout Patterns

### Sidebar chat

```tsx
function AppWithChat() {
  return (
    <div className="flex h-screen">
      <main className="flex-1">{/* App content */}</main>
      <aside className="w-[400px] border-l">
        <ChatKit control={control} className="h-full" />
      </aside>
    </div>
  );
}
```

### Floating widget

```tsx
function FloatingChat() {
  const [open, setOpen] = useState(false);
  const { control } = useChatKit({ api: { url: "/api/chatkit", domainKey: "app" } });

  return (
    <>
      <button
        onClick={() => setOpen(!open)}
        className="fixed bottom-4 right-4 z-50 rounded-full bg-indigo-600 p-4 text-white shadow-lg"
      >
        {open ? "Close" : "Chat"}
      </button>
      {open && (
        <div className="fixed bottom-20 right-4 z-50 w-[380px] h-[520px] rounded-xl shadow-2xl overflow-hidden">
          <ChatKit control={control} className="h-full w-full" />
        </div>
      )}
    </>
  );
}
```

### Full-page chat

```tsx
<ChatKit control={control} className="h-screen w-full" />
```

## Widgets

ChatKit supports rich interactive widgets streamed from the server. Widgets render inside the chat as interactive UI elements.

### Widget namespace components

Available under `Widgets.*`:
- **Layout**: `Box`, `Row`, `Col`, `Card`, `Divider`
- **Forms**: `Input`, `Textarea`, `Select`, `Checkbox`, `RadioGroup`, `DatePicker`
- **Display**: `Badge`, `Icon`, `Image`, `Markdown`, `ListView`
- **Text**: `Title`, `Caption`, `Label`

### Server-side widget streaming

```python
from chatkit.widgets import Card, Row, Col, Title, Caption, Badge

@function_tool()
async def show_search_results(ctx: RunContextWrapper[AgentContext], query: str):
    results = await search(query)
    cards = []
    for r in results:
        cards.append(
            Card(children=[
                Title(text=r["title"]),
                Caption(text=r["snippet"]),
                Badge(text=r["category"]),
            ])
        )
    return Col(children=cards)
```

## Actions

Actions let the server define interactive buttons/forms that the user can click:

```python
from chatkit.types import ActionEvent

# Stream an action prompt to the user
yield ActionEvent(
    item_id=msg_id,
    actions=[
        {"id": "yes", "label": "Yes, proceed"},
        {"id": "no", "label": "No, cancel"},
    ],
)
```

## File Uploads

### Frontend configuration

```tsx
const { control } = useChatKit({
  api: { url: "/api/chatkit", domainKey: "app" },
  fileUpload: {
    enabled: true,
    maxFileSize: 10 * 1024 * 1024,  // 10MB
    accept: [".pdf", ".txt", ".md", ".png", ".jpg"],
  },
});
```

### Backend AttachmentStore

Implement `AttachmentStore` to handle upload URLs:

```python
from chatkit.store import AttachmentStore

class S3AttachmentStore(AttachmentStore[MyRequestContext]):
    async def create_attachment(self, attachment, context):
        # Generate presigned upload URL
        upload_url = await generate_s3_presigned_url(attachment.id)
        attachment.upload_url = upload_url
        return attachment

    async def delete_attachment(self, attachment_id, context):
        await delete_s3_object(attachment_id)
```

## Thread Browsing

Enable thread history sidebar:

```tsx
const { control } = useChatKit({
  api: { url: "/api/chatkit", domainKey: "app" },
  threads: {
    enabled: true,  // Shows thread list sidebar
  },
});
```

The backend `load_threads()` store method handles pagination automatically.

## Next.js Integration

### API route proxy

```ts
// app/api/chatkit/route.ts
export async function POST(request: Request) {
  const body = await request.arrayBuffer();
  const response = await fetch("http://localhost:8000/chatkit", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      "X-User-Id": getUserIdFromSession(request),
    },
    body,
  });
  return new Response(response.body, {
    headers: { "Content-Type": response.headers.get("Content-Type") || "application/json" },
  });
}
```

### Client component

```tsx
"use client";
import { ChatKit, useChatKit } from "@openai/chatkit-react";

export function ChatWidget() {
  const { control } = useChatKit({
    api: { url: "/api/chatkit", domainKey: "my-app" },
  });
  return <ChatKit control={control} className="h-[600px]" />;
}
```

## Docusaurus Integration

For embedding ChatKit in a Docusaurus site:

### As a React component

```tsx
// src/components/ChatWidget/index.tsx
import React, { useState } from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";

function ChatInner() {
  const { ChatKit, useChatKit } = require("@openai/chatkit-react");
  const { control } = useChatKit({
    api: { url: "https://your-api.example.com/chatkit", domainKey: "textbook" },
  });

  return <ChatKit control={control} className="h-[500px] w-full rounded-lg" />;
}

export default function ChatWidget() {
  const [open, setOpen] = useState(false);
  return (
    <BrowserOnly>
      {() => (
        <>
          <button onClick={() => setOpen(!open)}>Ask AI</button>
          {open && <ChatInner />}
        </>
      )}
    </BrowserOnly>
  );
}
```

### Swizzle into layout

Add to `src/theme/Root.js` for site-wide floating chat:

```tsx
import React from "react";
import ChatWidget from "../components/ChatWidget";

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```
