<script setup>
import { RouterLink, RouterView } from 'vue-router'
</script>

<template

>
  <div style="widht: 100%">

  <grid-layout
    :layout.sync="layout"
    :col-num="40"
    :row-height="10"
    :is-draggable="true"
    :is-resizable="true"
    :is-mirrored="false"
    :vertical-compact="false"
    :margin="[10, 10]"
    :use-css-transforms="true"
    :preventCollision="false"
    :autoSize="false"
  >
    <grid-item v-for="item in layout"
        :x="item.x"
        :y="item.y"
        :w="item.w"
        :h="item.h"
        :i="item.i"
        :key="item.i">

      <img id="image_sub" v-if="item.i === '2'" />
      <div id="button_view" v-if="item.i === '3'">
        <table id="key_button_table">
          <tr class="key_button_table_row" >
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('load_left')"
                @mouseup="control_button_up('load_left')"
              >load left</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('unload_left')"
                @mouseup="control_button_up('unload_left')"
              >unload left</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('standby')"
                @mouseup="control_button_up('standby')"
              >standby</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('unload_right')"
                @mouseup="control_button_up('unload_right')"
              >unload right</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('load_right')"
                @mouseup="control_button_up('load_right')"
              >load right</n-button></td>
          </tr>
          <tr class="key_button_table_row" >
            <td class="key_button_table_cell" colspan="5"></td>
          </tr>
          <tr class="key_button_table_row" >
            <td class="key_button_table_cell"></td>
            <td class="key_button_table_cell"></td>
            <td class="key_button_table_cell">
              <n-button 
                class="key_but"
                @mousedown="control_button_down('up')"
                @mouseup="control_button_up('up')"
              >⇧</n-button></td>
            <td class="key_button_table_cell"></td>
            <td class="key_button_table_cell"></td>
          </tr>
          <tr class="key_button_table_row" >
            <td class="key_button_table_cell"></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('left')"
                @mouseup="control_button_up('left')"
              >⇦</n-button>
            </td>
            <td class="key_button_table_cell">
              <n-button 
                class="key_but"
                @mousedown="control_button_down('down')"
                @mouseup="control_button_up('down')"
              >⇩</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('right')"
                @mouseup="control_button_up('right')"
              >⇨</n-button></td>
            <td class="key_button_table_cell"></td>
          </tr>
        </table>

        
              
      </div>

      <n-table v-if="item.i === '0'" :bordered="true" :single-line="false">
          <thead>
            <tr>
              <th>Abandon</th>
              <th>Abormal</th>
              <th>Abolish</th>
              <th>...</th>
              <th>It's hard to learn words</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>放弃</td>
              <td>反常的</td>
              <td>彻底废除</td>
              <td>...</td>
              <td><n-button class="key_but">delete</n-button></td>
            </tr>
            <tr>
              <td>...</td>
              <td>...</td>
              <td>...</td>
              <td>...</td>
              <td>...</td>
            </tr>
          </tbody>
        </n-table>
      

    </grid-item>
    </grid-layout>
  
  </div>

</template>

<script>
import { GridLayout, GridItem } from 'vue3-grid-layout-next';
import { NButton, NTable } from 'naive-ui'
import ros from './mixins/ros.js';


export default {
  mixins: [ros],
  components: {GridLayout, GridItem, NTable, NButton},
  mounted() {
    document.addEventListener('keydown', (e) => {
      switch (e.code) {
        case 'ArrowLeft': case 'KeyA':
          this.control_button_down('left');
          break;
        case 'ArrowRight': case 'KeyD':
          this.control_button_down('right');
          break;
        case 'ArrowUp': case 'KeyW':
          this.control_button_down('up');
          break;
        case 'ArrowDown': case 'KeyS':
          this.control_button_down('down');
          break;
        default:
          break;
      }

    });
    document.addEventListener('keyup', (e) => {
      switch (e.code) {
        case 'ArrowLeft': case 'KeyA':
          this.control_button_up('left');
          break;
        case 'ArrowRight': case 'KeyD':
          this.control_button_up('right');
          break;
        case 'ArrowUp': case 'KeyW':
          this.control_button_up('up');
          break;
        case 'ArrowDown': case 'KeyS':
          this.control_button_up('down');
          break;
        default:
          break;
      }
    });
  },
  data () {
    return {
      layout: [
        {"x":0,"y":0,"w":20,"h":23,"i":"0", static: false},
        {"x":30,"y":0,"w":10,"h":23,"i":"1", static: false},
        // {"x":0,"y":0,"w":20,"h":23,"i":"2", static: false},
        {"x":27,"y":33,"w":13,"h":13,"i":"3", static: false},
      ],
      control_buttons: {
        up: false,
        down: false,
        left: false,
        right: false,
        load_left: false,
        load_right: false,
        unload_left: false,
        unload_right: false
      }
   }
  },
  methods: {
    control_button_down(button) {
      this.control_buttons[button] = true;
    },
    control_button_up(button) {
      this.control_buttons[button] = false;
    }
  }
}
</script>

<style scoped>
.lcol {
  height: inherit;
  border: 1px solid grey;
}
.layout {
  background-color: #ddd;
}
.test {
  background-color: #ddd;
}
.vue-grid-item {
  width: 150px;
  text-align: center;
  border: 1px solid #ddd;
  border-radius: 10px;
  /* margin: 10px 0; */
  /* padding: 10px; */
}

#image_sub {
  width: inherit;
  height: inherit;
}

.key_but {
  width: 100%;
  height: 100%;
  overflow: hidden;
  overflow-wrap: break-word;
  word-break: break-all;
}

#button_view {
  padding: 20px;
  width: 100%;
  height: 100%;
}

#key_button_table {
  width: inherit;
  height: inherit;
  
  /* border: 1px solid grey; */
}

  
/* .key_button_table_cell {
  border: 1px solid green;
} */

.light-green {
  height: auto;
  background-color: rgba(0, 128, 0, 0.12);
}
.green {
  height: auto;
  background-color: rgba(0, 128, 0, 0.24);
}
</style>
